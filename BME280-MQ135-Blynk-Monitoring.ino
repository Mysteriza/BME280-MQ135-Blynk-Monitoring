#define BLYNK_TEMPLATE_ID   "TMPLxyk260wi"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <EEPROM.h>

// -------- Pins & Blynk VPins --------
#define MQ135_PIN   A0
#define VP_TEMP     V0
#define VP_HUM      V1
#define VP_STATUS   V2
#define VP_BUTTON   V3
#define VP_TIME     V4
#define VP_MQ_RAW   V5
#define VP_IAQ      V6
#define VP_IAQ_STR  V7
#define VP_PRESS    V8
#define VP_ALT      V9

// -------- Credentials (fill yours) --------
char auth[] = "mnBmj5aj0LEKUdghiHnG81HKlt1aDFMR";
char ssid[] = "Kosan bu nata";
char pass[] = "immodium";

// -------- Objects --------
Adafruit_BME280 bme;
BlynkTimer timer;
WidgetRTC rtc;

// -------- Timing (ms) --------
const uint32_t T_SENSOR   = 2000;    // sensor tick
const uint32_t T_PUBLISH  = 5000;    // push to Blynk
const uint32_t T_NETCHK   = 5000;    // WiFi/Blynk health
const uint32_t WARMUP_MS  = 300000;  // 5 minutes

// -------- Filtering --------
const int   MQ_SAMPLES   = 9;        // median-of-N
const float EMA_ALPHA    = 0.08f;    // MQ135 EMA
const float P_EMA_ALPHA  = 0.10f;    // pressure EMA for stable altitude

// -------- Compensation (conservative + slew-limit) --------
const float T_REF  = 25.0f;
const float RH_REF = 50.0f;
const float K_T    = -0.004f;        // ~ -0.4% per °C
const float K_RH   =  0.001f;        // ~ +0.1% per %RH
const float COMP_MIN = 0.85f;
const float COMP_MAX = 1.15f;

// -------- IAQ mapping (piecewise, calmer & meaningful) --------
// Bands are absolute relative deviation |dev| = |(value - baseline)/baseline|
struct Band { float dev_hi; float idx_lo; float idx_hi; };
const Band IAQ_BANDS[] = {
  // dev    ->  IAQ range
  {0.02f,    0.0f,   50.0f},   // 0–2%  -> 0–50   (Very Good)
  {0.05f,   50.0f,  100.0f},   // 2–5%  -> 50–100 (Good)
  {0.10f,  100.0f,  150.0f},   // 5–10% -> 100–150 (Fair)
  {0.20f,  150.0f,  200.0f},   // 10–20%-> 150–200 (Poor)
  {0.40f,  200.0f,  300.0f},   // 20–40%-> 200–300
  {0.80f,  300.0f,  400.0f},   // 40–80%-> 300–400
  {9e9f,   400.0f,  500.0f}    // >80%  -> 400–500 (cap)
};
const float DEV_DEADBAND = 0.010f;    // ignore <1% dev

// -------- Baseline drift (very conservative) --------
const uint32_t DRIFT_INTERVAL_MS = 1800000; // 30 min
const float    DRIFT_STEP_FRAC   = 0.0003f; // 0.03% per interval
const float    DRIFT_GATE_DEV    = 0.02f;   // drift only when dev < 2%

// -------- Altitude settings (pressure-only) --------
const float SEA_LEVEL_HPA = 1012.25f;       // fixed sea-level pressure (per request)
const float ALT_SLEW_MAX  = 2.0f;           // max 2 m change per tick

// -------- EEPROM layout --------
struct Persist {
  uint32_t magic;     // 0xA135B135
  uint16_t version;   // 1
  float    adc0;      // baseline
  uint32_t checksum;  // simple sum
};
const uint32_t MAGIC = 0xA135B135;
const uint16_t VER   = 1;

// -------- Globals --------
float emaADC = NAN;
float baselineADC = NAN;
bool  calibrated = false;
bool  warming = true;
uint32_t t_start = 0;
uint32_t lastDrift = 0;
char bufMsg[64];

const int WBUF_MAX = 180; // 180*2s ≈ 6min (we stop at 5min)
float warmBuf[WBUF_MAX];
int   warmCnt = 0;

float lastTemp= NAN, lastHum= NAN;
float lastPress= NAN, pressEMA = NAN;
float lastAlt= NAN;
float lastADCraw = NAN, lastIAQ = NAN;
String lastIAQStr = "Unknown";

// -------- Utils --------
static uint32_t sum32(const uint8_t* p, size_t n) { uint32_t s=0; for(size_t i=0;i<n;i++) s+=p[i]; return s; }

void eepromLoad() {
  EEPROM.begin(512);
  Persist p; EEPROM.get(0,p);
  if (p.magic==MAGIC && p.version==VER) {
    uint32_t cs = p.checksum; p.checksum = 0;
    if (cs == sum32((uint8_t*)&p, sizeof(Persist)) && isfinite(p.adc0) && p.adc0>0.0f) {
      baselineADC = p.adc0; calibrated = true; warming = false;
    }
  }
}
void eepromSave(float adc0) {
  Persist p{MAGIC, VER, adc0, 0};
  p.checksum = sum32((uint8_t*)&p, sizeof(Persist));
  EEPROM.put(0,p); EEPROM.commit();
}

// -------- Net/Blynk health --------
void netTick() {
  if (WiFi.status() != WL_CONNECTED) { WiFi.begin(ssid, pass); return; }
  if (!Blynk.connected()) { Blynk.connect(2000); }
}

// -------- BME280 init --------
bool initBME() { return bme.begin(0x76) || bme.begin(0x77); }

// -------- Median-of-N ADC --------
int readADCMedian(uint8_t pin) {
  int v[MQ_SAMPLES];
  for (int i=0;i<MQ_SAMPLES;i++){ v[i]=analogRead(pin); delay(2); }
  for (int i=1;i<MQ_SAMPLES;i++){ int k=v[i], j=i-1; while(j>=0 && v[j]>k){v[j+1]=v[j]; j--;} v[j+1]=k; }
  return v[MQ_SAMPLES/2];
}

// -------- Compensation (with slew-limit) --------
float compFactor(float tC, float rh) {
  static float prev = 1.0f;
  float cf = 1.0f + K_T*(tC - T_REF) + K_RH*(rh - RH_REF);
  if (cf < COMP_MIN) cf = COMP_MIN; if (cf > COMP_MAX) cf = COMP_MAX;
  const float MAX_SLEW = 0.01f; // max 1% per tick
  float delta = cf - prev;
  if      (delta >  MAX_SLEW) cf = prev + MAX_SLEW;
  else if (delta < -MAX_SLEW) cf = prev - MAX_SLEW;
  prev = cf; return cf;
}

// -------- IAQ mapping (piecewise) --------
float iaqIndexFromDev(float dev) {
  if (dev < 0) dev = -dev;
  // deadband: ignore micro deviations
  dev = (dev > DEV_DEADBAND) ? (dev - DEV_DEADBAND) : 0.0f;

  float prev_hi = 0.0f, prev_idx = 0.0f;
  for (size_t i=0;i<sizeof(IAQ_BANDS)/sizeof(IAQ_BANDS[0]); ++i) {
    float hi = IAQ_BANDS[i].dev_hi;
    float lo = prev_hi;
    float frac = 0.0f;
    if (dev <= hi) {
      float span = hi - lo; if (span < 1e-6f) frac = 1.0f;
      else frac = (dev - lo)/span;
      float idx = IAQ_BANDS[i].idx_lo + frac*(IAQ_BANDS[i].idx_hi - IAQ_BANDS[i].idx_lo);
      if (idx < 0) idx = 0; if (idx > 500) idx = 500;
      return idx;
    }
    prev_hi = hi; prev_idx = IAQ_BANDS[i].idx_hi;
  }
  return 500.0f;
}
const char* iaqLabel(float idx){
  if (idx <= 50)  return "Very Good";
  if (idx <= 100) return "Good";
  if (idx <= 150) return "Fair";
  if (idx <= 200) return "Poor";
  return "Very Poor";
}

// -------- Baseline drift (only when very clean & stable) --------
void tryDrift(float devAbs) {
  if (!calibrated) return;
  if (millis() - lastDrift < DRIFT_INTERVAL_MS) return;
  lastDrift = millis();
  if (!(devAbs < DRIFT_GATE_DEV && isfinite(emaADC) && emaADC>0.0f)) return;
  if (!(isfinite(lastIAQ) && lastIAQ <= 100.0f)) return; // only when Good or better
  float delta = DRIFT_STEP_FRAC * baselineADC;
  baselineADC += (emaADC > baselineADC) ? delta : -delta;
  eepromSave(baselineADC);
}

// -------- Altitude from pressure only (stable) --------
float altitudeFromPressure(float press_hpa, float seaLevel_hpa) {
  // Standard barometric formula (no temperature dependency here)
  float alt = 44330.0f * (1.0f - powf(press_hpa / seaLevel_hpa, 0.1903f));
  return alt;
}

// -------- Sensor tick --------
void sensorTick() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;   // hPa
  if (isnan(t) || isnan(h) || isnan(p)) {
    Blynk.virtualWrite(VP_STATUS, "BME280 read failed");
    return;
  }

  // Pressure EMA for stable altitude
  if (isnan(pressEMA)) pressEMA = p;
  else pressEMA = P_EMA_ALPHA*p + (1.0f - P_EMA_ALPHA)*pressEMA;

  // Raw ADC → median → EMA
  int adc = readADCMedian(MQ135_PIN);
  if (adc <= 0 || adc >= 1023) {
    snprintf(bufMsg, sizeof(bufMsg), "MQ135 saturated: %d", adc);
    Blynk.virtualWrite(VP_STATUS, bufMsg);
    return;
  }
  float x = (float)adc;
  if (isnan(emaADC)) emaADC = x; else emaADC = EMA_ALPHA*x + (1.0f-EMA_ALPHA)*emaADC;

  uint32_t elapsed = millis() - t_start;
  if (warming) {
    if (warmCnt < WBUF_MAX) warmBuf[warmCnt++] = emaADC;
    if (elapsed >= WARMUP_MS) {
      // median baseline from warmup buffer
      for (int i=1;i<warmCnt;i++){ float k=warmBuf[i]; int j=i-1; while(j>=0 && warmBuf[j]>k){warmBuf[j+1]=warmBuf[j]; j--; } warmBuf[j+1]=k; }
      baselineADC = warmBuf[warmCnt/2];
      if (!isfinite(baselineADC) || baselineADC<=0) baselineADC = emaADC;
      eepromSave(baselineADC);
      warming = false; calibrated = true;
      snprintf(bufMsg, sizeof(bufMsg), "Warm-up done. Baseline=%.1f", baselineADC);
      Blynk.virtualWrite(VP_STATUS, bufMsg);
    } else {
      int secs = (WARMUP_MS - elapsed)/1000;
      snprintf(bufMsg, sizeof(bufMsg), "Warming up... %ds", secs);
      Blynk.virtualWrite(VP_STATUS, bufMsg);
    }
  } else {
    // Light T/RH compensation (conservative + slew-limit)
    float cf = compFactor(t, h);
    float adcComp = emaADC * cf;

    // Relative deviation vs baseline
    float dev = (baselineADC>1.0f) ? (adcComp - baselineADC)/baselineADC : 0.0f;
    float devAbs = fabsf(dev);

    // IAQ (piecewise)
    float iaq = iaqIndexFromDev(devAbs);
    const char* lbl = iaqLabel(iaq);

    // Stable altitude from pressure EMA
    float alt_now = altitudeFromPressure(pressEMA, SEA_LEVEL_HPA);
    // Slew-limit altitude to avoid jitter
    if (!isfinite(lastAlt)) lastAlt = alt_now;
    float dAlt = alt_now - lastAlt;
    if      (dAlt >  ALT_SLEW_MAX) lastAlt += ALT_SLEW_MAX;
    else if (dAlt < -ALT_SLEW_MAX) lastAlt -= ALT_SLEW_MAX;
    else                           lastAlt  = alt_now;
    // Round to nearest meter
    lastAlt = roundf(lastAlt);

    // Store latest for publish
    lastTemp=t; lastHum=h; lastPress=pressEMA;
    lastADCraw = emaADC; lastIAQ = iaq; lastIAQStr = lbl;

    // Gentle drift only when really clean & stable
    tryDrift(devAbs);
  }
}

// -------- Publish tick --------
void publishTick() {
  if (!Blynk.connected()) return;
  if (isfinite(lastTemp))   Blynk.virtualWrite(VP_TEMP, lastTemp);
  if (isfinite(lastHum))    Blynk.virtualWrite(VP_HUM, lastHum);
  if (isfinite(lastPress))  Blynk.virtualWrite(VP_PRESS, lastPress);
  if (isfinite(lastAlt))    Blynk.virtualWrite(VP_ALT, lastAlt);
  if (!warming && calibrated) {
    if (isfinite(lastADCraw)) Blynk.virtualWrite(VP_MQ_RAW, lastADCraw);
    if (isfinite(lastIAQ))    Blynk.virtualWrite(VP_IAQ, lastIAQ);
    Blynk.virtualWrite(VP_IAQ_STR, lastIAQStr);
    Blynk.virtualWrite(VP_STATUS, "OK");
  }
  if (timeStatus() == timeSet) {
    char ts[6]; snprintf(ts, sizeof(ts), "%02d:%02d", hour(), minute());
    Blynk.virtualWrite(VP_TIME, ts);
  }
}

// -------- Blynk handlers --------
BLYNK_CONNECTED() {
  rtc.begin();
  Blynk.virtualWrite(VP_STATUS, "Connected to Blynk");
}
BLYNK_WRITE(VP_BUTTON) {
  if (param.asInt()==1) { publishTick(); Blynk.virtualWrite(VP_BUTTON, 0); }
}

// -------- Setup/Loop --------
void setup() {
  eepromLoad();
  if (!initBME()) { for(;;){ delay(800); } }  // fatal halt
  WiFi.mode(WIFI_STA); WiFi.begin(ssid, pass);
  Blynk.config(auth);

  t_start = millis(); lastDrift = millis();
  timer.setInterval(T_SENSOR,  sensorTick);
  timer.setInterval(T_PUBLISH, publishTick);
  timer.setInterval(T_NETCHK,  netTick);
}
void loop() {
  Blynk.run();
  timer.run();
}

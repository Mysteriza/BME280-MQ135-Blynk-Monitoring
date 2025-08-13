/***** ESP8266 + MQ135 (A0) + BME280 (I2C) + Blynk (non-blocking) *****
 * Goals:
 * - 5-min auto warm-up + baseline capture (no manual calibration).
 * - IAQ Index (0..500) based on relative deviation from baseline (no fake "CO2 ppm").
 * - Light T/RH compensation (conservative).
 * - Median + EMA filtering; EEPROM baseline persistence; gentle baseline drift.
 * - Non-blocking WiFi/Blynk (config/connect), periodic publish.
 *********************************************************************/

#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <EEPROM.h>

// --- Pins & Blynk VPins ---
#define MQ135_PIN A0
#define VP_TEMP V0
#define VP_HUM V1
#define VP_STATUS V2
#define VP_BUTTON V3
#define VP_TIME V4
#define VP_MQ_RAW V5
#define VP_IAQ V6
#define VP_IAQ_STR V7
#define VP_PRESS V8
#define VP_ALT V9

// --- Credentials ---
char auth[] = "";
char ssid[] = "";
char pass[] = "";

// --- Objects ---
Adafruit_BME280 bme;
BlynkTimer timer;
WidgetRTC rtc;

// --- Timing ---
const uint32_t T_SENSOR = 2000;
const uint32_t T_PUBLISH = 10000;
const uint32_t T_NETCHK = 5000;
const uint32_t WARMUP_MS = 300000;

// --- Filtering ---
const int MQ_SAMPLES = 9;
const float EMA_ALPHA = 0.20f;

// --- Compensation ---
const float T_REF = 25.0f;
const float RH_REF = 50.0f;
const float K_T = -0.010f;
const float K_RH = 0.003f;
const float COMP_MIN = 0.5f;
const float COMP_MAX = 1.5f;

// --- IAQ mapping ---
const float DEV_REF = 0.25f;
const float GAMMA = 1.315f;

// --- Baseline drift ---
const uint32_t DRIFT_INTERVAL_MS = 600000;
const float DRIFT_STEP_FRAC = 0.001f;
const float DRIFT_GATE_DEV = 0.05f;

// --- EEPROM layout ---
struct Persist
{
  uint32_t magic;
  uint16_t version;
  float adc0;
  uint32_t checksum;
};
const uint32_t MAGIC = 0xA135B135;
const uint16_t VER = 1;
const size_t EE_SZ = sizeof(Persist);

// --- Globals ---
float emaADC = NAN;
float baselineADC = NAN;
bool calibrated = false;
bool warming = true;
uint32_t t_start = 0;
uint32_t lastDrift = 0;
char bufMsg[64];
const int WBUF_MAX = 180;
float warmBuf[WBUF_MAX];
int warmCnt = 0;

// --- Utils ---
static uint32_t sum32(const uint8_t *p, size_t n)
{
  uint32_t s = 0;
  for (size_t i = 0; i < n; i++)
    s += p[i];
  return s;
}

void eepromLoad()
{
  EEPROM.begin(512);
  Persist p;
  EEPROM.get(0, p);
  if (p.magic == MAGIC && p.version == VER)
  {
    uint32_t cs = p.checksum;
    p.checksum = 0;
    uint32_t calc = sum32((uint8_t *)&p, sizeof(Persist));
    if (calc == cs && isfinite(p.adc0) && p.adc0 > 0.0f)
    {
      baselineADC = p.adc0;
      calibrated = true;
    }
  }
}

void eepromSave(float adc0)
{
  Persist p;
  p.magic = MAGIC;
  p.version = VER;
  p.adc0 = adc0;
  p.checksum = 0;
  p.checksum = sum32((uint8_t *)&p, sizeof(Persist));
  EEPROM.put(0, p);
  EEPROM.commit();
}

// --- Net/Blynk health ---
void netTick()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, pass);
    return;
  }
  if (!Blynk.connected())
  {
    Blynk.connect(2000);
  }
}

// --- BME280 init ---
bool initBME()
{
  if (bme.begin(0x76))
    return true;
  if (bme.begin(0x77))
    return true;
  return false;
}

// --- Median-of-N ADC read ---
int readADCMedian(uint8_t pin)
{
  int v[MQ_SAMPLES];
  for (int i = 0; i < MQ_SAMPLES; i++)
  {
    v[i] = analogRead(pin);
    delay(2);
  }
  for (int i = 1; i < MQ_SAMPLES; i++)
  {
    int key = v[i], j = i - 1;
    while (j >= 0 && v[j] > key)
    {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  return v[MQ_SAMPLES / 2];
}

// --- Compensation factor ---
float compFactor(float tC, float rh)
{
  float cf = 1.0f + K_T * (tC - T_REF) + K_RH * (rh - RH_REF);
  if (cf < COMP_MIN)
    cf = COMP_MIN;
  if (cf > COMP_MAX)
    cf = COMP_MAX;
  return cf;
}

// --- IAQ mapping ---
float iaqIndexFromDev(float dev)
{
  if (dev < 0)
    dev = -dev;
  float r = dev / DEV_REF;
  if (r < 0)
    r = 0;
  if (r > 1)
    r = 1;
  float idx = 500.0f * powf(r, GAMMA);
  if (idx < 0)
    idx = 0;
  if (idx > 500)
    idx = 500;
  return idx;
}

const char *iaqLabel(float idx)
{
  if (idx <= 50)
    return "Very Good";
  if (idx <= 100)
    return "Good";
  if (idx <= 150)
    return "Fair";
  if (idx <= 200)
    return "Poor";
  return "Very Poor";
}

// --- Gentle baseline drift ---
void tryDrift(float devAbs)
{
  if (!calibrated)
    return;
  if (millis() - lastDrift < DRIFT_INTERVAL_MS)
    return;
  lastDrift = millis();
  if (devAbs < DRIFT_GATE_DEV && isfinite(emaADC) && emaADC > 0.0f)
  {
    float delta = DRIFT_STEP_FRAC * baselineADC;
    if (emaADC > baselineADC)
      baselineADC = baselineADC + delta;
    else
      baselineADC = baselineADC - delta;
    eepromSave(baselineADC);
  }
}

// --- Sensor tick ---
float lastTemp = NAN, lastHum = NAN, lastPress = NAN, lastAlt = NAN;
float lastADCraw = NAN, lastIAQ = NAN;
String lastIAQStr = "Unknown";

void sensorTick()
{
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  float a = bme.readAltitude(1013.25f);

  if (isnan(t) || isnan(h) || isnan(p) || isnan(a))
  {
    snprintf(bufMsg, sizeof(bufMsg), "BME280 read failed");
    Blynk.virtualWrite(VP_STATUS, bufMsg);
    return;
  }

  int adc = readADCMedian(MQ135_PIN);
  if (adc <= 0 || adc >= 1023)
  {
    snprintf(bufMsg, sizeof(bufMsg), "MQ135 saturated: %d", adc);
    Blynk.virtualWrite(VP_STATUS, bufMsg);
    return;
  }

  float x = (float)adc;
  if (isnan(emaADC))
    emaADC = x;
  else
    emaADC = EMA_ALPHA * x + (1.0f - EMA_ALPHA) * emaADC;

  uint32_t elapsed = millis() - t_start;
  if (warming)
  {
    if (warmCnt < WBUF_MAX)
      warmBuf[warmCnt++] = emaADC;
    if (elapsed >= WARMUP_MS)
    {
      for (int i = 1; i < warmCnt; i++)
      {
        float key = warmBuf[i];
        int j = i - 1;
        while (j >= 0 && warmBuf[j] > key)
        {
          warmBuf[j + 1] = warmBuf[j];
          j--;
        }
        warmBuf[j + 1] = key;
      }
      baselineADC = warmBuf[warmCnt / 2];
      if (!isfinite(baselineADC) || baselineADC <= 0)
        baselineADC = emaADC;
      eepromSave(baselineADC);
      warming = false;
      calibrated = true;
      snprintf(bufMsg, sizeof(bufMsg), "Warm-up done. Baseline=%.1f", baselineADC);
      Blynk.virtualWrite(VP_STATUS, bufMsg);
    }
    else
    {
      int secs = (WARMUP_MS - elapsed) / 1000;
      snprintf(bufMsg, sizeof(bufMsg), "Warming up... %ds", secs);
      Blynk.virtualWrite(VP_STATUS, bufMsg);
    }
  }
  else
  {
    float cf = compFactor(t, h);
    float adcComp = emaADC * cf;
    float dev = (baselineADC > 1.0f) ? (adcComp - baselineADC) / baselineADC : 0.0f;
    float devAbs = fabsf(dev);
    float iaq = iaqIndexFromDev(devAbs);
    const char *lbl = iaqLabel(iaq);

    lastTemp = t;
    lastHum = h;
    lastPress = p;
    lastAlt = a;
    lastADCraw = emaADC;
    lastIAQ = iaq;
    lastIAQStr = lbl;

    tryDrift(devAbs);
  }
}

// --- Publish tick ---
void publishTick()
{
  if (!Blynk.connected())
    return;

  if (isfinite(lastTemp))
    Blynk.virtualWrite(VP_TEMP, lastTemp);
  if (isfinite(lastHum))
    Blynk.virtualWrite(VP_HUM, lastHum);
  if (isfinite(lastPress))
    Blynk.virtualWrite(VP_PRESS, lastPress);
  if (isfinite(lastAlt))
    Blynk.virtualWrite(VP_ALT, lastAlt);

  if (!warming && calibrated)
  {
    if (isfinite(lastADCraw))
      Blynk.virtualWrite(VP_MQ_RAW, lastADCraw);
    if (isfinite(lastIAQ))
      Blynk.virtualWrite(VP_IAQ, lastIAQ);
    Blynk.virtualWrite(VP_IAQ_STR, lastIAQStr);
    Blynk.virtualWrite(VP_STATUS, "OK");
  }

  if (timeStatus() == timeSet)
  {
    char ts[6];
    snprintf(ts, sizeof(ts), "%02d:%02d", hour(), minute());
    Blynk.virtualWrite(VP_TIME, ts);
  }
}

// --- Blynk handlers ---
BLYNK_CONNECTED()
{
  rtc.begin();
  Blynk.virtualWrite(VP_STATUS, "Connected to Blynk");
}

BLYNK_WRITE(VP_BUTTON)
{
  if (param.asInt() == 1)
  {
    publishTick();
    Blynk.virtualWrite(VP_BUTTON, 0);
  }
}

// --- Setup/Loop ---
void setup()
{
  eepromLoad();
  if (isfinite(baselineADC) && baselineADC > 0)
  {
    warming = false;
    calibrated = true;
  }

  if (!initBME())
  {
    for (;;)
    {
      delay(800);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(auth);

  t_start = millis();
  lastDrift = millis();

  timer.setInterval(T_SENSOR, sensorTick);
  timer.setInterval(T_PUBLISH, publishTick);
  timer.setInterval(T_NETCHK, netTick);
}

void loop()
{
  Blynk.run();
  timer.run();
}

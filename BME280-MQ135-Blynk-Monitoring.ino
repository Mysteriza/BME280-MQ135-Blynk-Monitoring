/***** ESP8266 + MQ135 (A0) + BME280 (I2C) + Blynk (non-blocking)
 * Minimal telemetry: Temperature, Humidity, Pressure, MQ135 Raw
 * - No altitude, no IAQ index, no IAQ status
 * - Median + EMA filtering for MQ135; light EMA for pressure
 * - Fast, simple, and robust networking (non-blocking)
 *********************************************************************/

#define BLYNK_TEMPLATE_ID   "TMPLxyk260wi"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

// -------- Pins & Blynk VPins --------
#define MQ135_PIN   A0
#define VP_TEMP     V0
#define VP_HUM      V1
#define VP_STATUS   V2
#define VP_BUTTON   V3
#define VP_TIME     V4
#define VP_MQ_RAW   V5
#define VP_PRESS    V8

// -------- Credentials --------
char auth[] = "mnBmj5aj0LEKUdghiHnG81HKlt1aDFMR";
char ssid[] = "Kosan bu nata";
char pass[] = "immodium";

// -------- Objects --------
Adafruit_BME280 bme;
BlynkTimer timer;
WidgetRTC rtc;

// -------- Timing (ms) --------
const uint32_t T_SENSOR   = 1500;    // faster sensor tick
const uint32_t T_PUBLISH  = 3000;    // faster publish
const uint32_t T_NETCHK   = 4000;    // network health

// -------- Filtering --------
const int   MQ_SAMPLES   = 9;        // median-of-N
const float EMA_ALPHA    = 0.08f;    // MQ135 EMA
const float P_EMA_ALPHA  = 0.10f;    // pressure EMA

// -------- State --------
float lastTemp  = NAN, lastHum = NAN;
float lastPress = NAN, pressEMA = NAN;
float lastADCraw = NAN;
char  bufMsg[48];

// -------- Utils --------
bool initBME() { return bme.begin(0x76) || bme.begin(0x77); }

int readADCMedian(uint8_t pin) {
  int v[MQ_SAMPLES];
  for (int i=0;i<MQ_SAMPLES;i++){ v[i]=analogRead(pin); delay(2); }
  for (int i=1;i<MQ_SAMPLES;i++){ int k=v[i], j=i-1; while(j>=0 && v[j]>k){v[j+1]=v[j]; j--;} v[j+1]=k; }
  return v[MQ_SAMPLES/2];
}

void netTick() {
  if (WiFi.status() != WL_CONNECTED) { WiFi.begin(ssid, pass); return; }
  if (!Blynk.connected()) { Blynk.connect(2000); }
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

  // Pressure EMA (stable display)
  if (isnan(pressEMA)) pressEMA = p;
  else pressEMA = P_EMA_ALPHA*p + (1.0f - P_EMA_ALPHA)*pressEMA;

  // MQ135 median -> EMA
  int adc = readADCMedian(MQ135_PIN);
  if (adc <= 0 || adc >= 1023) {
    snprintf(bufMsg, sizeof(bufMsg), "MQ135 saturated: %d", adc);
    Blynk.virtualWrite(VP_STATUS, bufMsg);
    return;
  }
  float x = (float)adc;
  if (isnan(lastADCraw)) lastADCraw = x;
  else lastADCraw = EMA_ALPHA*x + (1.0f-EMA_ALPHA)*lastADCraw;

  // Store latest
  lastTemp = t; lastHum = h; lastPress = pressEMA;
}

// -------- Publish tick --------
void publishTick() {
  if (!Blynk.connected()) return;

  if (isfinite(lastTemp))   Blynk.virtualWrite(VP_TEMP,  lastTemp);
  if (isfinite(lastHum))    Blynk.virtualWrite(VP_HUM,   lastHum);
  if (isfinite(lastPress))  Blynk.virtualWrite(VP_PRESS, lastPress);
  if (isfinite(lastADCraw)) Blynk.virtualWrite(VP_MQ_RAW,lastADCraw);

  Blynk.virtualWrite(VP_STATUS, "OK");

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
  if (!initBME()) { for(;;){ delay(800); } }   // fatal: halt with soft wait
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(auth);

  timer.setInterval(T_SENSOR,  sensorTick);
  timer.setInterval(T_PUBLISH, publishTick);
  timer.setInterval(T_NETCHK,  netTick);
}

void loop() {
  Blynk.run();
  timer.run();
}

#define BLYNK_TEMPLATE_ID   "TMPLxyk260wi"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

#define MQ135_PIN   A0
#define VP_TEMP     V0
#define VP_HUM      V1
#define VP_STATUS   V2
#define VP_BUTTON   V3
#define VP_TIME     V4
#define VP_MQ_RAW   V5
#define VP_PRESS    V8

char auth[] = "";
char ssid[] = "";
char pass[] = "";

Adafruit_BME280 bme;
BlynkTimer timer;
WidgetRTC rtc;

const uint32_t T_SENSOR   = 2000;
const uint32_t T_PUBLISH  = 4000;
const uint32_t T_NETCHK   = 2000;

const int   MQ_SAMPLES   = 9;
const float EMA_ALPHA    = 0.08f;
const float P_EMA_ALPHA  = 0.10f;

float lastTemp  = NAN;
float lastHum = NAN;
float lastPress = NAN;
float pressEMA = NAN;
float lastADCraw = NAN;

uint32_t wifiBackoffMs  = 1000;
uint32_t blynkBackoffMs = 1000;
const uint32_t WIFI_BACKOFF_MIN   = 1000;
const uint32_t WIFI_BACKOFF_MAX   = 30000;
const uint32_t BLYNK_BACKOFF_MIN  = 1000;
const uint32_t BLYNK_BACKOFF_MAX  = 30000;
uint32_t nextWifiAttemptAt  = 0;
uint32_t nextBlynkAttemptAt = 0;

bool initBME() {
  return bme.begin(0x76) || bme.begin(0x77);
}

int readADCMedian(uint8_t pin) {
  int v[MQ_SAMPLES];
  for (int i=0;i<MQ_SAMPLES;i++){
    v[i]=analogRead(pin);
    delay(2);
  }
  for (int i=1;i<MQ_SAMPLES;i++){
    int k=v[i];
    int j=i-1;
    while(j>=0 && v[j]>k){
      v[j+1]=v[j];
      j--;
    }
    v[j+1]=k;
  }
  return v[MQ_SAMPLES/2];
}

inline uint32_t nowMs() {
  return millis();
}

void scheduleNextWifiAttempt(bool success) {
  if (success) {
    wifiBackoffMs = WIFI_BACKOFF_MIN;
  }
  else {
    wifiBackoffMs = min(WIFI_BACKOFF_MAX, wifiBackoffMs << 1);
  }
  nextWifiAttemptAt = nowMs() + wifiBackoffMs;
}

void scheduleNextBlynkAttempt(bool success) {
  if (success) {
    blynkBackoffMs = BLYNK_BACKOFF_MIN;
  }
  else {
    blynkBackoffMs = min(BLYNK_BACKOFF_MAX, blynkBackoffMs << 1);
  }
  nextBlynkAttemptAt = nowMs() + blynkBackoffMs;
}

void netTick() {
  if (WiFi.status() != WL_CONNECTED) {
    if (nowMs() >= nextWifiAttemptAt) {
      WiFi.begin(ssid, pass);
      scheduleNextWifiAttempt(false);
    }
    return;
  } else {
    if (wifiBackoffMs != WIFI_BACKOFF_MIN) {
      scheduleNextWifiAttempt(true);
    }
  }

  if (!Blynk.connected()) {
    if (nowMs() >= nextBlynkAttemptAt) {
      bool ok = Blynk.connect(1500);
      scheduleNextBlynkAttempt(ok);
      if (ok) {
        Blynk.virtualWrite(VP_STATUS, "Connected to Blynk");
      }
    }
  } else {
    if (blynkBackoffMs != BLYNK_BACKOFF_MIN) {
      scheduleNextBlynkAttempt(true);
    }
  }
}

void sensorTick() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;

  if (isnan(t) || isnan(h) || isnan(p)) {
    Blynk.virtualWrite(VP_STATUS, "BME280 read failed");
    return;
  }

  if (isnan(pressEMA)) {
    pressEMA = p;
  }
  else {
    pressEMA = P_EMA_ALPHA*p + (1.0f - P_EMA_ALPHA)*pressEMA;
  }

  int adc = readADCMedian(MQ135_PIN);
  if (adc <= 0 || adc >= 1023) {
    Blynk.virtualWrite(VP_STATUS, "MQ135 saturated");
    return;
  }

  float x = (float)adc;
  if (isnan(lastADCraw)) {
    lastADCraw = x;
  }
  else {
    lastADCraw = EMA_ALPHA*x + (1.0f-EMA_ALPHA)*lastADCraw;
  }

  lastTemp = t;
  lastHum = h;
  lastPress = pressEMA;
}

void publishTick() {
  if (!Blynk.connected()) {
    return;
  }

  if (isfinite(lastTemp)) {
    Blynk.virtualWrite(VP_TEMP,  lastTemp);
  }
  if (isfinite(lastHum)) {
    Blynk.virtualWrite(VP_HUM,   lastHum);
  }
  if (isfinite(lastPress)) {
    Blynk.virtualWrite(VP_PRESS, lastPress);
  }
  if (isfinite(lastADCraw)) {
    Blynk.virtualWrite(VP_MQ_RAW,lastADCraw);
  }

  Blynk.virtualWrite(VP_STATUS, "OK");

  if (timeStatus() == timeSet) {
    char ts[9];
    snprintf(ts, sizeof(ts), "%02d:%02d:%02d", hour(), minute(), second());
    Blynk.virtualWrite(VP_TIME, ts);
  }
}

BLYNK_CONNECTED() {
  rtc.begin();
  Blynk.virtualWrite(VP_STATUS, "Connected to Blynk");
}

BLYNK_WRITE(VP_BUTTON) {
  if (param.asInt()==1) {
    publishTick();
    Blynk.virtualWrite(VP_BUTTON, 0);
  }
}

void setup() {
  if (!initBME()) {
    while(true) {
      delay(800);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);

  Blynk.config(auth);

  scheduleNextWifiAttempt(false);
  scheduleNextBlynkAttempt(false);

  timer.setInterval(T_SENSOR,  sensorTick);
  timer.setInterval(T_PUBLISH, publishTick);
  timer.setInterval(T_NETCHK,  netTick);
}

void loop() {
  Blynk.run();
  timer.run();
}

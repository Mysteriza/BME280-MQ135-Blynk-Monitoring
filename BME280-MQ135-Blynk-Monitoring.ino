#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BME280.h> // Library for BME280 sensor
#include <TimeLib.h>
#include <WidgetRTC.h>

// Pin definitions
// BME280 uses I2C (SDA=D2, SCL=D1 on NodeMCU ESP8266 by default), no explicit pin define needed
#define MQ135_PIN A0        // Analog pin for MQ-135 sensor
#define LED_PIN D0          // Onboard LED pin (active low)

// WiFi and Blynk credentials
char auth[] = "";
char ssid[] = "";
char pass[] = "";

// Initialize objects
Adafruit_BME280 bme; // Create a BME280 object
BlynkTimer timer;
WidgetRTC rtc;

// Constants
const float R0 = 400.0; // Initial baseline resistance (will adjust naturally over 24-48 hours)
const float A_COEFFICIENT = 110.0; // Constant for CO2 (adjust based on calibration)
const float B_COEFFICIENT = -2.5;   // Exponent for CO2 (adjust based on calibration)
const float TEMP_BASELINE = 27.5;   // Adjusted to average room temperature based on your room
const float HUMIDITY_BASELINE = 60.0; // Adjusted to average humidity based on your room
const float TEMP_SENSITIVITY = 0.04; // Sensitivity to temperature change (approx. 4% per °C)
const float HUMIDITY_SENSITIVITY = 0.02; // Sensitivity to humidity change (approx. 2% per %)
const unsigned long RECONNECT_INTERVAL = 5000;       // 5 seconds for WiFi
const unsigned long BLYNK_RECONNECT_INTERVAL = 10000; // 10 seconds for Blynk
#define SEALEVEL_PRESSURE_HPA 1013.25 // Standard sea level pressure in hPa (millibars)

// Reconnection variables
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastBlynkReconnectAttempt = 0;

// LED control variables
unsigned long ledStartTime = 0;
bool ledActive = false;

// Check and reconnect WiFi
void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiReconnectAttempt > RECONNECT_INTERVAL) {
    lastWiFiReconnectAttempt = millis();
    // Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
    Blynk.virtualWrite(V2, "WiFi Disconnected - Reconnecting...");
    timer.setTimeout(5000L, []() {
      if (WiFi.status() == WL_CONNECTED) {
        // Serial.println("WiFi reconnected successfully!");
        Blynk.virtualWrite(V2, "WiFi Reconnected!");
      }
    });
  }
}

// Check and reconnect Blynk
void checkBlynkConnection() {
  if (!Blynk.connected() && millis() - lastBlynkReconnectAttempt > BLYNK_RECONNECT_INTERVAL) {
    lastBlynkReconnectAttempt = millis();
    // Serial.println("Blynk disconnected. Attempting to reconnect...");
    Blynk.disconnect();
    Blynk.begin(auth, ssid, pass);
    Blynk.virtualWrite(V2, "Blynk Disconnected - Reconnecting...");
  }
}

// Read MQ-135 with averaging
float readMQ135() {
  const int SAMPLES = 5;
  int total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogRead(MQ135_PIN);
    delay(10);
  }
  return total / (float)SAMPLES;
}

// Calculate compensated gas concentration in ppm
float calculateCompensatedGas(float raw_gas, float temperature, float humidity) {
  // Adjust raw gas reading based on temperature and humidity
  float tempFactor = 1.0 + ((temperature - TEMP_BASELINE) * TEMP_SENSITIVITY);
  float humidFactor = 1.0 + ((humidity - HUMIDITY_BASELINE) * HUMIDITY_SENSITIVITY);
  float adjustedRs = raw_gas * tempFactor * humidFactor;

  // Convert to gas concentration (ppm) using simplified MQ-135 model
  // Serial.print("Debug - adjustedRs: "); Serial.print(adjustedRs);
  // Serial.print(", ratio: "); Serial.println(adjustedRs / R0);
  float ratio = adjustedRs / R0;
  float concentration = A_COEFFICIENT * pow(ratio, B_COEFFICIENT);

  // Limit to realistic range for CO2 (10-1000 ppm)
  return constrain(concentration, 10.0, 1000.0);
}

// Get air quality status based on gas concentration
String getAirQualityStatus(float compensated_gas) {
  if (compensated_gas <= 400) return "Very Good";   // ~400 ppm is typical outdoor CO2
  if (compensated_gas <= 600) return "Good";
  if (compensated_gas <= 800) return "Fair";
  if (compensated_gas <= 1000) return "Poor";
  return "Very Poor";
}

// Check if it's night time (19:00 to 07:00)
bool isNightTime() {
  if (!timeStatus() == timeSet) return false; // Default to daytime if time not synced
  int currentHour = hour();
  return (currentHour >= 19 || currentHour < 7);
}

// Handle LED blinking or solid on (non-blocking)
void updateLED(bool sensorFailure, bool poorAirQuality) {
  if (isNightTime() || ledActive) return; // Skip if night mode or LED already active

  if (sensorFailure) {
    ledActive = true;
    ledStartTime = millis();
    timer.setTimer(500L, []() {
      static int blinkCount = 0;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
      blinkCount++;
      if (blinkCount >= 6) { // 3 full blinks (6 toggles)
        digitalWrite(LED_PIN, HIGH);
        ledActive = false;
        blinkCount = 0;
        return false; // Stop timer
      }
      return true; // Continue timer
    }, 6);
  } else if (poorAirQuality) {
    ledActive = true;
    digitalWrite(LED_PIN, LOW);
    timer.setTimeout(3000L, []() {
      digitalWrite(LED_PIN, HIGH);
      ledActive = false;
    });
  }
}

// Send sensor data to Blynk
void sendSensorData() {
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa (millibars)
  float altitude = bme.readAltitude(SEALEVEL_PRESSURE_HPA); // Calculate altitude based on sea level pressure

  if (isnan(temperature) || isnan(humidity) || isnan(pressure) || isnan(altitude)) {
    // Serial.println("Failed to read from BME280 sensor!");
    Blynk.virtualWrite(V2, "Failed to read from BME280 sensor!");
    updateLED(true, false);
    return;
  }

  float raw_gas = readMQ135();
  if (raw_gas < 0 || raw_gas > 1023) {
    // Serial.println("Failed to read from MQ-135 sensor!");
    Blynk.virtualWrite(V2, "Invalid MQ-135 sensor reading!");
    updateLED(true, false);
    return;
  }

  float compensated_gas = calculateCompensatedGas(raw_gas, temperature, humidity);
  String airQualityStatus = getAirQualityStatus(compensated_gas);

  updateLED(false, compensated_gas > 800); // Adjusted threshold for poor air quality

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V8, pressure);   // New Virtual Pin for Pressure
  Blynk.virtualWrite(V9, altitude);   // New Virtual Pin for Altitude
  Blynk.virtualWrite(V5, raw_gas);
  Blynk.virtualWrite(V6, compensated_gas);
  Blynk.virtualWrite(V7, airQualityStatus);

  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", hour(), minute());
  Blynk.virtualWrite(V4, timeStr);

  Blynk.virtualWrite(V2, "Latest Update!");
}

void setup() {
  // Serial.begin(115200); // Uncomment for debugging
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn off LED initially (active low)

  // Initialize BME280
  // Default I2C address is 0x76 or 0x77.
  // bme.begin() returns true on success, false otherwise.
  if (!bme.begin(0x76)) { // Try 0x76 first, or 0x77 if 0x76 doesn't work
    // Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1); // Halt if sensor not found
  }

  Blynk.begin(auth, ssid, pass);
  rtc.begin();

  timer.setInterval(20000L, sendSensorData);
  timer.setInterval(5000L, checkWiFi);
  timer.setInterval(10000L, checkBlynkConnection);
}

void loop() {
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V3) {
  // This virtual pin can be used to manually trigger sensor data send from Blynk app.
  if (param.asInt() == 1) {
    sendSensorData();
    Blynk.virtualWrite(V3, 0); // Reset the button in Blynk app
  }
}

BLYNK_CONNECTED() {
  rtc.begin();
  // Serial.println("Connected to Blynk!");
  Blynk.virtualWrite(V2, "Connected to Blynk!");
}
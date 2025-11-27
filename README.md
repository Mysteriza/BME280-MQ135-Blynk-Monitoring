# 🌫️ ESP8266 Air Quality & Environment Monitor with BME280 & MQ-135

This project is a smart air quality and environment monitoring system based on the **NodeMCU ESP8266**, utilizing the **MQ-135** gas sensor and **BME280** for temperature, humidity, pressure, and altitude readings. It integrates with **Blynk IoT Platform** for real-time remote monitoring and status updates.

## [Web App Monitoring](https://github.com/Mysteriza/WebApp-IoT-Monitoring)
---

## 📦 Features

- Reads and compensates **air quality (CO2 estimate)** based on MQ-135 sensor.
- Measures **temperature**, **humidity**, **barometric pressure**, and **altitude** using BME280.
- **WiFi and Blynk auto-reconnect logic** for robust uptime.
- Real-time updates to the **Blynk app dashboard**.
- Virtual button to **manually trigger data upload** from Blynk app.

---

## 🧰 Hardware Requirements

| Component             | Description                            |
|----------------------|----------------------------------------|
| NodeMCU ESP8266      | Wi-Fi enabled microcontroller board     |
| MQ-135 Gas Sensor    | For air quality (CO2, NH3, etc.)        |
| BME280 Sensor        | For temperature, humidity, pressure     |
| Breadboard & Wires   | For prototyping connections             |
| Power Supply         | USB or external 5V                      |

> **Note:** BME280 uses I2C by default (SDA = D2, SCL = D1 on NodeMCU).

---

## 🔧 Circuit Diagram

| Sensor       | NodeMCU Pin |
|--------------|-------------|
| MQ-135 (AO)  | A0          |
| BME280 (SDA) | D2          |
| BME280 (SCL) | D1          |

---

## 🛠️ Installation & Setup

1. **Install Arduino IDE** (if not already installed)
2. Add **ESP8266 board support** via Board Manager.
3. Install required libraries:
   - `Blynk`
   - `Adafruit BME280`
   - `Adafruit Unified Sensor`
   - `TimeLib`

4. Configure your credentials in the code:
   ```cpp
   char auth[] = "YourBlynkAuthToken";
   char ssid[] = "YourWiFiSSID";
   char pass[] = "YourWiFiPassword";

5. Upload the sketch to your NodeMCU ESP8266.
6. Open Serial Monitor (baud rate: 9600 or 115200 depending on your code) to verify initialization and connection.
7. After boot, the system will:
   - Warm up the MQ-135 sensor for 5 minutes.
   - Capture a baseline air quality reading.
   - Start sending compensated and filtered readings to Blynk every few seconds.

## ⚙️ How It Works
1. Warm-up & Baseline Capture
   - On power-up, the MQ-135 sensor is given a 5-minute warm-up time to stabilize.
   - A median value from readings during warm-up is stored as the baseline for air quality comparison.
2. Sensor Reading & Compensation
   - MQ-135 provides raw ADC values proportional to gas concentration.
   - BME280 provides temperature, humidity, and pressure.
   - A compensation factor adjusts MQ-135 readings based on temperature and humidity.
   - The system calculates a relative deviation from the baseline and converts it into an IAQ Index (0-500).
3. Filtering
   - Median filtering reduces noise spikes.
   - EMA (Exponential Moving Average) smooths out short-term fluctuations.
4. Blynk Integration
   - Data is sent to the Blynk dashboard for remote monitoring.
   - A manual update button on the Blynk app can trigger an immediate data push.
   - The device auto-reconnects to WiFi and Blynk if the connection drops.

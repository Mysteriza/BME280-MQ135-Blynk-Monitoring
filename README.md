# 🌫️ ESP8266 Air Quality & Environment Monitor with BME280 & MQ-135

This project is a smart air quality and environment monitoring system based on the **NodeMCU ESP8266**, utilizing the **MQ-135** gas sensor and **BME280** for temperature, humidity, pressure, and altitude readings. It integrates with **Blynk IoT Platform** for real-time remote monitoring and status updates.

---

## 📦 Features

- Reads and compensates **air quality (CO2 estimate)** based on MQ-135 sensor.
- Measures **temperature**, **humidity**, **barometric pressure**, and **altitude** using BME280.
- **Night mode LED control** to avoid disturbances during sleep.
- **WiFi and Blynk auto-reconnect logic** for robust uptime.
- Real-time updates to the **Blynk app dashboard**.
- Visual LED alerts:
  - Blinks on sensor failure.
  - Solid light on poor air quality.
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
| LED          | D0 (onboard)|

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

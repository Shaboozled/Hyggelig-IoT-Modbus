#include <Arduino.h>
#include <WiFi.h>
#include <math.h>

#include "ModbusServerWiFi.h"

// ============================================================================
// ESP32 #1 - SENSOR UNIT / MODBUS TCP SERVER
//
// This firmware connects to the Wi-Fi access point created by ESP32 #2
// (the gateway) and exposes three holding registers over Modbus TCP.
//
// It is configured to match the gateway firmware supplied earlier:
//   Gateway AP SSID:       ESP32_LOCAL
//   Gateway AP password:   ChangeThisAPPassword
//   Sensor ESP32 IP:       192.168.50.2
//   Gateway AP IP:         192.168.50.1
//   Modbus TCP port:       502
//   Modbus unit ID:        1
//   Function code:         0x03 (Read Holding Registers)
//   Start address:         0 (zero-based; commonly documented as 40001)
//   Register count:        3
//
// IMPORTANT: readSensorValues() currently returns DEMO values. Replace that
// function with reads from your actual sensors when you decide which sensors
// and libraries to use.
// ============================================================================

// ----------------------------- Wi-Fi settings -------------------------------
static const char *WIFI_SSID = "ESP32_LOCAL";
static const char *WIFI_PASSWORD = "ChangeThisAPPassword";

static const IPAddress DEVICE_IP(192, 168, 50, 2);
static const IPAddress GATEWAY_IP(192, 168, 50, 1);
static const IPAddress SUBNET_MASK(255, 255, 255, 0);

static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

// ---------------------------- Modbus settings -------------------------------
static constexpr uint16_t MODBUS_TCP_PORT = 502;
static constexpr uint8_t MODBUS_UNIT_ID = 1;
static constexpr uint8_t FC_READ_HOLDING_REGISTERS = 0x03;
static constexpr uint8_t MODBUS_MAX_CLIENTS = 2;
static constexpr uint32_t MODBUS_CLIENT_TIMEOUT_MS = 15000;

// These addresses are zero-based Modbus PDU offsets.
// Address 0 is commonly shown as holding register 40001 in device manuals.
static constexpr uint16_t REG_TEMPERATURE = 0;
static constexpr uint16_t REG_HUMIDITY = 1;
static constexpr uint16_t REG_PRESSURE = 2;
static constexpr uint16_t REGISTER_COUNT = 3;

// ----------------------------- Sensor settings -----------------------------
static constexpr uint32_t SENSOR_UPDATE_INTERVAL_MS = 1000;

struct SensorValues {
  float temperatureC;
  float humidityPercent;
  float pressureHpa;
};

// Shared holding-register values.
// Register 0: signed temperature, tenths of degrees C (e.g. 235 = 23.5 C)
// Register 1: unsigned humidity, tenths of percent (e.g. 552 = 55.2 %)
// Register 2: unsigned pressure, tenths of hPa (e.g. 10132 = 1013.2 hPa)
static uint16_t holdingRegisters[REGISTER_COUNT] = {0, 0, 0};
static portMUX_TYPE registerMux = portMUX_INITIALIZER_UNLOCKED;

static ModbusServerWiFi modbusServer;
static bool modbusServerStarted = false;
static uint32_t lastSensorUpdateMs = 0;
static uint32_t lastWifiRetryMs = 0;

// ---------------------------- Sensor placeholder ---------------------------
SensorValues readSensorValues() {
  // DEMO VALUES ONLY.
  // Replace these three values with measurements from your real sensors.
  // For example:
  //   values.temperatureC = yourTemperatureSensor.readCelsius();
  //   values.humidityPercent = yourHumiditySensor.readHumidity();
  //   values.pressureHpa = yourPressureSensor.readPressure() / 100.0f;
  return {23.5f, 55.2f, 1013.2f};
}

// Convert a signed floating-point measurement to signed tenths.
int16_t encodeSignedTenths(float value) {
  if (!isfinite(value)) {
    return 0;
  }

  long scaled = lroundf(value * 10.0f);
  if (scaled < -32768L) scaled = -32768L;
  if (scaled > 32767L) scaled = 32767L;
  return static_cast<int16_t>(scaled);
}

// Convert a non-negative floating-point measurement to unsigned tenths.
uint16_t encodeUnsignedTenths(float value) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0;
  }

  unsigned long scaled = static_cast<unsigned long>(lroundf(value * 10.0f));
  if (scaled > 65535UL) scaled = 65535UL;
  return static_cast<uint16_t>(scaled);
}

void updateHoldingRegisters() {
  const SensorValues values = readSensorValues();

  // Store signed temperature as its 16-bit two's-complement representation.
  const int16_t temperatureRaw = encodeSignedTenths(values.temperatureC);
  const uint16_t humidityRaw = encodeUnsignedTenths(values.humidityPercent);
  const uint16_t pressureRaw = encodeUnsignedTenths(values.pressureHpa);

  // The Modbus server callback runs in a separate task, so protect shared data.
  portENTER_CRITICAL(&registerMux);
  holdingRegisters[REG_TEMPERATURE] = static_cast<uint16_t>(temperatureRaw);
  holdingRegisters[REG_HUMIDITY] = humidityRaw;
  holdingRegisters[REG_PRESSURE] = pressureRaw;
  portEXIT_CRITICAL(&registerMux);

  Serial.printf("Sensor values updated: T=%.1f C, RH=%.1f %%, P=%.1f hPa\n",
                temperatureRaw / 10.0f,
                humidityRaw / 10.0f,
                pressureRaw / 10.0f);
}

// -------------------------- Modbus request handler -------------------------
// eModbus passes the Modbus PDU in this layout:
//   byte 0: unit/server ID
//   byte 1: function code
//   byte 2-3: starting register address (big-endian, zero-based)
//   byte 4-5: quantity of registers (big-endian)
ModbusMessage handleReadHoldingRegisters(ModbusMessage request) {
  uint16_t startAddress = 0;
  uint16_t quantity = 0;
  ModbusMessage response;

  if (request.size() < 6) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    return response;
  }

  request.get(2, startAddress);
  request.get(4, quantity);

  // Modbus FC03 permits 1..125 registers per request. This device exposes 3.
  if (quantity == 0 || quantity > 125 ||
      startAddress >= REGISTER_COUNT ||
      quantity > (REGISTER_COUNT - startAddress)) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  uint16_t registerSnapshot[REGISTER_COUNT];
  portENTER_CRITICAL(&registerMux);
  for (uint16_t i = 0; i < REGISTER_COUNT; ++i) {
    registerSnapshot[i] = holdingRegisters[i];
  }
  portEXIT_CRITICAL(&registerMux);

  // Build FC03 response: [unit ID][function][byte count][register words...]
  response.add(request.getServerID(),
               request.getFunctionCode(),
               static_cast<uint8_t>(quantity * 2));

  for (uint16_t i = 0; i < quantity; ++i) {
    response.add(registerSnapshot[startAddress + i]);
  }

  return response;
}

// ------------------------------- Wi-Fi --------------------------------------
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  if (!WiFi.config(DEVICE_IP, GATEWAY_IP, SUBNET_MASK)) {
    Serial.println("WARNING: Could not apply the static IP configuration.");
  }

  Serial.printf("Connecting to gateway AP '%s'...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = now;
    Serial.println("Wi-Fi disconnected; attempting to reconnect...");
    WiFi.reconnect();
  }
}

void startModbusServerWhenConnected() {
  if (modbusServerStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  Serial.printf("Connected. Sensor ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Gateway IP: %s\n", WiFi.gatewayIP().toString().c_str());

  // Register the FC03 worker for unit ID 1, matching the gateway's request.
  modbusServer.registerWorker(MODBUS_UNIT_ID,
                             FC_READ_HOLDING_REGISTERS,
                             &handleReadHoldingRegisters);

  Serial.printf("Starting Modbus TCP server on port %u...\n", MODBUS_TCP_PORT);
  modbusServerStarted = modbusServer.start(
      MODBUS_TCP_PORT,
      MODBUS_MAX_CLIENTS,
      MODBUS_CLIENT_TIMEOUT_MS);

  if (modbusServerStarted) {
    Serial.println("Modbus TCP server is running.");
  } else {
    Serial.println("Could not start the Modbus TCP server.");
  }
}

// ----------------------------- Arduino setup/loop --------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("ESP32 sensor unit - Modbus TCP server");
  Serial.println("--------------------------------------");

  updateHoldingRegisters();
  lastSensorUpdateMs = millis();

  startWiFi();
}

void loop() {
  maintainWiFi();
  startModbusServerWhenConnected();

  const uint32_t now = millis();
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    updateHoldingRegisters();
  }

  static uint32_t lastStatusPrintMs = 0;
  if (modbusServerStarted && now - lastStatusPrintMs >= 10000) {
    lastStatusPrintMs = now;
    Serial.printf("Modbus server active clients: %u\n", modbusServer.activeClients());
  }

  delay(10);
}

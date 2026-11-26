#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <DHT.h>
#include "ModbusServerWiFi.h"

// ESP32 #1: DHT11 / Modbus TCP server
// Wiring: DHT11 VCC->3V3, DATA->GPIO4, GND->GND
// Blue measurement LED: GPIO25 -> 220 ohm -> anode, cathode -> GND
// Red error LED:        GPIO27 -> 220 ohm -> anode, cathode -> GND

static const char *WIFI_SSID = "ESP32_LOCAL";
static const char *WIFI_PASSWORD = "ChangeThisAPPassword";
static const IPAddress DEVICE_IP(192, 168, 50, 2);
static const IPAddress GATEWAY_IP(192, 168, 50, 1);
static const IPAddress SUBNET_MASK(255, 255, 255, 0);
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

static constexpr uint16_t MODBUS_TCP_PORT = 502;
static constexpr uint8_t MODBUS_UNIT_ID = 1;
static constexpr uint8_t FC_READ_HOLDING_REGISTERS = 0x03;
static constexpr uint8_t MODBUS_MAX_CLIENTS = 2;
static constexpr uint32_t MODBUS_CLIENT_TIMEOUT_MS = 15000;

// Zero-based holding registers: 40001, 40002, 40003.
static constexpr uint16_t REG_TEMPERATURE = 0; // signed tenths of Celsius
static constexpr uint16_t REG_HUMIDITY = 1;    // unsigned tenths of %RH
static constexpr uint16_t REG_STATUS = 2;      // 0 OK, 1 read error, 2 not measured yet
static constexpr uint16_t REGISTER_COUNT = 3;

static constexpr uint8_t DHT_DATA_PIN = 4;
static constexpr uint8_t BLUE_LED_PIN = 25;
static constexpr uint8_t RED_LED_PIN = 27;
static constexpr uint32_t SENSOR_UPDATE_INTERVAL_MS = 10000;
static constexpr uint32_t LED_BLINK_MS = 300;

DHT dht(DHT_DATA_PIN, DHT11);
static uint16_t holdingRegisters[REGISTER_COUNT] = {0, 0, 2};
static portMUX_TYPE registerMux = portMUX_INITIALIZER_UNLOCKED;
static ModbusServerWiFi modbusServer;
static bool modbusServerStarted = false;
static uint32_t lastSensorUpdateMs = 0;
static uint32_t lastWifiRetryMs = 0;
static uint32_t ledTurnOffAt = 0;
static bool blueLedOn = false;

int16_t encodeSignedTenths(float value) {
  long scaled = lroundf(value * 10.0f);
  if (scaled < -32768L) scaled = -32768L;
  if (scaled > 32767L) scaled = 32767L;
  return static_cast<int16_t>(scaled);
}

uint16_t encodeUnsignedTenths(float value) {
  if (value <= 0.0f) return 0;
  unsigned long scaled = static_cast<unsigned long>(lroundf(value * 10.0f));
  if (scaled > 65535UL) scaled = 65535UL;
  return static_cast<uint16_t>(scaled);
}

void updateHoldingRegisters() {
  // This light indicates an attempted measurement, not guaranteed success.
  digitalWrite(BLUE_LED_PIN, HIGH);
  blueLedOn = true;
  ledTurnOffAt = millis() + LED_BLINK_MS;

  const float temperatureC = dht.readTemperature();
  const float humidityPct = dht.readHumidity();
  const bool valid = isfinite(temperatureC) && isfinite(humidityPct) &&
                     humidityPct >= 0.0f && humidityPct <= 100.0f &&
                     temperatureC >= 0.0f && temperatureC <= 50.0f;

  portENTER_CRITICAL(&registerMux);
  if (valid) {
    holdingRegisters[REG_TEMPERATURE] =
        static_cast<uint16_t>(encodeSignedTenths(temperatureC));
    holdingRegisters[REG_HUMIDITY] = encodeUnsignedTenths(humidityPct);
    holdingRegisters[REG_STATUS] = 0;
  } else {
    // Retain last valid values. The status register marks them as stale.
    holdingRegisters[REG_STATUS] = 1;
  }
  portEXIT_CRITICAL(&registerMux);

  digitalWrite(RED_LED_PIN, valid ? LOW : HIGH);
  if (valid) {
    Serial.printf("DHT11 OK: T=%.1f C, RH=%.1f %%\n", temperatureC, humidityPct);
  } else {
    Serial.println("DHT11 FEJL: invalid or missing sensor reading");
  }
}

void updateLed() {
  if (blueLedOn && static_cast<int32_t>(millis() - ledTurnOffAt) >= 0) {
    digitalWrite(BLUE_LED_PIN, LOW);
    blueLedOn = false;
  }
}

// FC03: request = [unit ID][function][start address: 2][quantity: 2]
ModbusMessage handleReadHoldingRegisters(ModbusMessage request) {
  ModbusMessage response;
  if (request.size() < 6) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    return response;
  }

  uint16_t startAddress = 0;
  uint16_t quantity = 0;
  request.get(2, startAddress);
  request.get(4, quantity);
  if (quantity == 0 || quantity > 125 || startAddress >= REGISTER_COUNT ||
      quantity > (REGISTER_COUNT - startAddress)) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  uint16_t snapshot[REGISTER_COUNT];
  portENTER_CRITICAL(&registerMux);
  for (uint16_t i = 0; i < REGISTER_COUNT; ++i) snapshot[i] = holdingRegisters[i];
  portEXIT_CRITICAL(&registerMux);

  response.add(request.getServerID(), request.getFunctionCode(),
               static_cast<uint8_t>(quantity * 2));
  for (uint16_t i = 0; i < quantity; ++i) {
    response.add(snapshot[startAddress + i]);
  }
  return response;
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  if (!WiFi.config(DEVICE_IP, GATEWAY_IP, SUBNET_MASK)) {
    Serial.println("WARNING: Could not apply static IP configuration");
  }
  Serial.printf("Connecting to gateway AP '%s'...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  const uint32_t now = millis();
  if (now - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = now;
    Serial.println("Wi-Fi disconnected; retrying");
    WiFi.reconnect();
  }
}

void startModbusServerWhenConnected() {
  if (modbusServerStarted || WiFi.status() != WL_CONNECTED) return;
  Serial.printf("Sensor IP: %s\n", WiFi.localIP().toString().c_str());
  modbusServer.registerWorker(MODBUS_UNIT_ID, FC_READ_HOLDING_REGISTERS,
                             &handleReadHoldingRegisters);
  modbusServerStarted = modbusServer.start(MODBUS_TCP_PORT,
                                           MODBUS_MAX_CLIENTS,
                                           MODBUS_CLIENT_TIMEOUT_MS);
  Serial.println(modbusServerStarted ? "Modbus TCP server running" :
                                        "ERROR: Modbus server failed to start");
}

void setup() {
  Serial.begin(115200);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);
  // Until first successful read, error indicator is on (status = 2).
  digitalWrite(RED_LED_PIN, HIGH);
  dht.begin();
  Serial.println("ESP32 DHT11 Modbus TCP Server starting");
  startWiFi();
  // DHT11 requires a settling period after power-up.
  lastSensorUpdateMs = millis() - SENSOR_UPDATE_INTERVAL_MS + 2000;
}

void loop() {
  maintainWiFi();
  startModbusServerWhenConnected();

  const uint32_t now = millis();
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    updateHoldingRegisters();
  }
  updateLed();

  static uint32_t lastStatusPrintMs = 0;
  if (modbusServerStarted && now - lastStatusPrintMs >= 10000) {
    lastStatusPrintMs = now;
    Serial.printf("Modbus active clients: %u\n", modbusServer.activeClients());
  }
  delay(5);
}

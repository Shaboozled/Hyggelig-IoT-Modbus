#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include <PubSubClient.h>

// Set to 1 below to use TLS for a public/internet MQTT broker.
#define MQTT_USE_TLS 1

// eModbus: asynchronous Modbus TCP client
#include "ModbusClientTCP.h"
#include "ModbusMessage.h"

#if MQTT_USE_TLS
#include <WiFiClientSecure.h>
#else
#include <WiFiClient.h>
#endif

// -----------------------------------------------------------------------------
// USER CONFIGURATION - EDIT THESE VALUES
// -----------------------------------------------------------------------------

// Local Wi-Fi network created by THIS gateway ESP32.
static const char *AP_SSID = "ESP32_LOCAL";
static const char *AP_PASSWORD = "ChangeThisAPPassword"; // At least 8 characters
static const uint8_t AP_CHANNEL = 6; // AP+STA ultimately follows the upstream AP channel

// Wi-Fi network that provides internet access to this gateway.
static const char *UPLINK_SSID = "MotorolaEdge20";
static const char *UPLINK_PASSWORD = "projektlille";

// Static IP of the OTHER ESP32 (the Modbus TCP server) on this gateway's AP network.
// Configure the other ESP32 to use this address, subnet and gateway.
static const IPAddress MODBUS_SERVER_IP(192, 168, 50, 2);
static constexpr uint16_t MODBUS_PORT = 502;
static constexpr uint8_t MODBUS_UNIT_ID = 1;

// Modbus function 0x03 = Read Holding Registers.
// Protocol addresses are zero-based: address 0 is often labelled 40001 in manuals.
static constexpr uint16_t MODBUS_START_ADDRESS = 0;
static constexpr uint16_t MODBUS_REGISTER_COUNT = 3;
static constexpr uint32_t MODBUS_POLL_INTERVAL_MS = 2000;

// Register map expected from the other ESP32:
//   register 0: signed temperature in tenths of a degree C (e.g. 235 = 23.5 C)
//   register 1: unsigned humidity in tenths of a percent (e.g. 552 = 55.2 %)
//   register 2: unsigned pressure in tenths of hPa (e.g. 10132 = 1013.2 hPa)

// Mosquitto broker reachable through the UPLINK Wi-Fi network.
// Example for a broker on your LAN: host = PC/server LAN IP, port = 1883.
// Example for an internet-hosted broker: use its DNS name and normally TLS port 8883.
static const char *MQTT_HOST = "gruppe7.vps.webdock.cloud";
static constexpr uint16_t MQTT_PORT = 8883;
static const char *MQTT_TOPIC = "factory/machine01/sensors";
static const char *MQTT_CLIENT_ID = "ESP32_1"; // Must be unique for this broker
static const char *MQTT_USERNAME = ""; // Leave empty if broker has no username
static const char *MQTT_PASSWORD = ""; // Leave empty if broker has no password

// 0 = plain MQTT (suitable for an isolated local-network test only).
// 1 = MQTT over TLS (recommended for a broker reached over the public internet).

#if MQTT_USE_TLS
// Paste the PEM CA certificate that validates your broker's TLS certificate.
// Do not use setInsecure() for a production/public-internet connection.
static const char *MQTT_CA_CERT = R"PEM(
-----BEGIN CERTIFICATE-----
MIIDsTCCApmgAwIBAgIUFTKyiPD4pCMh1oDFaHMCaxrYFAkwDQYJKoZIhvcNAQEL
BQAwaDELMAkGA1UEBhMCREsxGjAYBgNVBAgMEVJlZ2lvbi1TeWRkYW5tYXJrMQ8w
DQYDVQQHDAZPZGVuc2UxDDAKBgNVBAoMA1VDTDEMMAoGA1UECwwDVUNMMRAwDgYD
VQQDDAdHcnVwcGU3MB4XDTI2MDkwMjExMDgxOFoXDTM2MDgzMDExMDgxOFowaDEL
MAkGA1UEBhMCREsxGjAYBgNVBAgMEVJlZ2lvbi1TeWRkYW5tYXJrMQ8wDQYDVQQH
DAZPZGVuc2UxDDAKBgNVBAoMA1VDTDEMMAoGA1UECwwDVUNMMRAwDgYDVQQDDAdH
cnVwcGU3MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAmaxhkJef8aiC
U5KjrmgDDZ/SxLYYM+PVbMoeBonGQmJMPki/tVArn0hqCDxrOGpEzlS9aDiizZHT
p2OPAWc5+Yqb1Kok2Esbp7yUUdrCfN7iDtHaGppt1BCVUOX+cksb/tQFLizhrfv4
4Ukxk2j0+sTxO0unNIbv8KqEDrl+YB+j2femFalOUU/W8EjTQHH9oizLo8kc9f28
XlM3I0EUUdJZ5NYLWksZLD7qHkVNDT7qcLxTgQWMjyB30GjKTFX5WSbLZMT5tCZ6
NUbTnEHuAnKAgvt0Dr/LkUaYMS1ZkBKIHRQKXfey/m87U74lqxrcwYJexpFxfRtL
1Kbkk9TpkwIDAQABo1MwUTAdBgNVHQ4EFgQUNEyiP6NLY4KNADHoqokR7SW8gnAw
HwYDVR0jBBgwFoAUNEyiP6NLY4KNADHoqokR7SW8gnAwDwYDVR0TAQH/BAUwAwEB
/zANBgkqhkiG9w0BAQsFAAOCAQEARm2Px/PcdW/2W52gmLUvKxmDKOq97uataX3U
BZVKsupiJJc9/tgBfQcj5WhJCPBQO6+ZLBk0oOSTDbnik4yocDrv6vwC/NNYuq18
ulp/16AH40vS5aVSBLHKEC58m1126c+SxCB34Inffl5DZvXTjLs1zmCy2rbFmnjz
7RzKLFJp0qvchGi8ddqGwqnawxJC7nSwPn+3o21ogBwBhWD45FWxmHPEw8BnPtLi
or/CVw6ngisBhyT1C+zRPjP6d/35kjxwBvV735WHeY5JAxO5VhPWMYHBEfxFlzD9
svNEmD46vDagMOs3qek5pjRsCIidzw81o7kSTg/g3Ii/YWHfMg==
-----END CERTIFICATE-----
)PEM";
#endif

static constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

// -----------------------------------------------------------------------------
// NETWORK AND CLIENT OBJECTS
// -----------------------------------------------------------------------------

static const IPAddress AP_IP(192, 168, 50, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

WiFiClient modbusTcpSocket;
ModbusClientTCP modbusClient(modbusTcpSocket, MODBUS_SERVER_IP, MODBUS_PORT);

#if MQTT_USE_TLS
WiFiClientSecure mqttTransport;
#else
WiFiClient mqttTransport;
#endif
PubSubClient mqttClient(mqttTransport);

// -----------------------------------------------------------------------------
// SHARED DATA (Modbus callback runs on the library's worker task)
// -----------------------------------------------------------------------------

struct SensorData {
  int16_t temperatureTenths;
  uint16_t humidityTenths;
  uint16_t pressureTenths;
  uint32_t sequence;
};

static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
static SensorData latestData = {};
static bool dataReady = false;
static bool modbusRequestPending = false;
static uint32_t nextRequestToken = 1;

static uint32_t lastModbusPollMs = 0;
static uint32_t lastMqttAttemptMs = 0;
static uint32_t lastWifiRetryMs = 0;

// -----------------------------------------------------------------------------
// MODBUS RESPONSE CALLBACK
// -----------------------------------------------------------------------------

void onModbusResponse(ModbusMessage message, uint32_t token) {
  (void)token;

  const Error modbusError = message.getError();

  if (modbusError != SUCCESS) {
    Serial.printf("Modbus response error: %u\n", static_cast<unsigned>(modbusError));
    portENTER_CRITICAL(&dataMux);
    modbusRequestPending = false;
    portEXIT_CRITICAL(&dataMux);
    return;
  }

  // FC03 response layout: [unit id][function][byte count][register data...]
  // Three 16-bit registers require 3 + (3 * 2) = 9 bytes in this message.
  if (message.getFunctionCode() != 0x03 || message.size() < 9) {
    Serial.println("Modbus response had an unexpected function or length.");
    portENTER_CRITICAL(&dataMux);
    modbusRequestPending = false;
    portEXIT_CRITICAL(&dataMux);
    return;
  }

  int16_t temperatureRaw = 0;
  uint16_t humidityRaw = 0;
  uint16_t pressureRaw = 0;

  // Data begins at byte offset 3 in the Modbus response message.
  message.get(3, temperatureRaw);
  message.get(5, humidityRaw);
  message.get(7, pressureRaw);

  portENTER_CRITICAL(&dataMux);
  latestData.temperatureTenths = temperatureRaw;
  latestData.humidityTenths = humidityRaw;
  latestData.pressureTenths = pressureRaw;
  latestData.sequence++;
  dataReady = true;
  modbusRequestPending = false;
  portEXIT_CRITICAL(&dataMux);

  Serial.printf("Modbus OK: T=%.1f C, RH=%.1f %%, P=%.1f hPa\n",
                temperatureRaw / 10.0f,
                humidityRaw / 10.0f,
                pressureRaw / 10.0f);
}

// -----------------------------------------------------------------------------
// WIFI
// -----------------------------------------------------------------------------

void startWifi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  if (!WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET)) {
    Serial.println("Could not configure the local AP IP settings.");
  }

  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 1)) {
    Serial.println("Failed to start the local Wi-Fi access point.");
  } else {
    Serial.printf("Local AP started: SSID=%s, IP=%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  // This station connection is ONLY for the gateway's own MQTT/internet traffic.
  WiFi.begin(UPLINK_SSID, UPLINK_PASSWORD);
  Serial.printf("Connecting to upstream Wi-Fi: %s\n", UPLINK_SSID);
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = now;
    Serial.println("Upstream Wi-Fi disconnected; requesting reconnect...");
    WiFi.reconnect();
  }
}

// -----------------------------------------------------------------------------
// MQTT
// -----------------------------------------------------------------------------

void maintainMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqttClient.connected()) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastMqttAttemptMs < MQTT_RECONNECT_INTERVAL_MS) {
    return;
  }
  lastMqttAttemptMs = now;

  Serial.printf("Connecting to MQTT broker %s:%u...\n", MQTT_HOST, MQTT_PORT);

  bool connected = false;
  if (strlen(MQTT_USERNAME) > 0) {
    connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
  } else {
    connected = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (connected) {
    Serial.println("MQTT connected.");
  } else {
    Serial.printf("MQTT connection failed, state=%d\n", mqttClient.state());
  }
}

void publishLatestData() {
  if (!mqttClient.connected()) {
    return;
  }

  SensorData snapshot;
  bool haveData = false;

  portENTER_CRITICAL(&dataMux);
  if (dataReady) {
    snapshot = latestData;
    haveData = true;
  }
  portEXIT_CRITICAL(&dataMux);

  if (!haveData) {
    return;
  }

  const float temperatureC = snapshot.temperatureTenths / 10.0f;
  const float humidityPct = snapshot.humidityTenths / 10.0f;
  const float pressureHpa = snapshot.pressureTenths / 10.0f;

  const time_t nowEpoch = time(nullptr);
  char payload[256];
  int payloadLength;

  if (nowEpoch > 1700000000) {
    payloadLength = snprintf(
        payload, sizeof(payload),
        "{\"gateway_id\":\"ESP32_GATEWAY_01\",\"source_id\":\"ESP32_SENSOR_01\","
        "\"temperature_c\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f,"
        "\"timestamp\":%lld}",
        temperatureC, humidityPct, pressureHpa, static_cast<long long>(nowEpoch));
  } else {
    payloadLength = snprintf(
        payload, sizeof(payload),
        "{\"gateway_id\":\"ESP32_GATEWAY_01\",\"source_id\":\"ESP32_SENSOR_01\","
        "\"temperature_c\":%.1f,\"humidity_pct\":%.1f,\"pressure_hpa\":%.1f,"
        "\"timestamp\":null}",
        temperatureC, humidityPct, pressureHpa);
  }

  if (payloadLength <= 0 || payloadLength >= static_cast<int>(sizeof(payload))) {
    Serial.println("Could not format MQTT payload.");
    return;
  }

  if (mqttClient.publish(MQTT_TOPIC, payload)) {
    Serial.printf("MQTT published to %s: %s\n", MQTT_TOPIC, payload);

    // Only clear the flag if no newer Modbus sample arrived during publishing.
    portENTER_CRITICAL(&dataMux);
    if (latestData.sequence == snapshot.sequence) {
      dataReady = false;
    }
    portEXIT_CRITICAL(&dataMux);
  } else {
    Serial.println("MQTT publish failed; latest sample will be retried.");
  }
}

// -----------------------------------------------------------------------------
// MODBUS POLLING
// -----------------------------------------------------------------------------

void pollModbusServer() {
  // Don't queue requests until the sensor ESP32 has joined this access point.
  if (WiFi.softAPgetStationNum() == 0) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastModbusPollMs < MODBUS_POLL_INTERVAL_MS) {
    return;
  }
  lastModbusPollMs = now;

  portENTER_CRITICAL(&dataMux);
  const bool alreadyPending = modbusRequestPending;
  if (!alreadyPending) {
    modbusRequestPending = true;
  }
  portEXIT_CRITICAL(&dataMux);

  if (alreadyPending) {
    return;
  }

  const uint32_t token = nextRequestToken++;
  const Error result = modbusClient.addRequest(
      token,
      MODBUS_UNIT_ID,
      0x03, // Read Holding Registers
      MODBUS_START_ADDRESS,
      MODBUS_REGISTER_COUNT);

  if (result != SUCCESS) {
    Serial.printf("Could not queue Modbus request, error=%u\n",
                  static_cast<unsigned>(result));
    portENTER_CRITICAL(&dataMux);
    modbusRequestPending = false;
    portEXIT_CRITICAL(&dataMux);
  }
}

// -----------------------------------------------------------------------------
// SETUP / LOOP
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 Modbus-to-MQTT gateway starting...");

  startWifi();

#if MQTT_USE_TLS
  mqttTransport.setCACert(MQTT_CA_CERT);
#endif

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);

  // Synchronize the system clock for a useful Unix timestamp in MQTT messages.
  // This does not block startup; timestamp remains null until time is available.
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  modbusClient.setTimeout(1500);
  modbusClient.onResponseHandler(onModbusResponse);
  modbusClient.begin();

  Serial.println("Gateway initialized.");
  Serial.printf("Modbus target: %s:%u\n",
                MODBUS_SERVER_IP.toString().c_str(), MODBUS_PORT);
}

void loop() {
  maintainWifi();
  maintainMqtt();

  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  pollModbusServer();
  publishLatestData();

  delay(5);
}

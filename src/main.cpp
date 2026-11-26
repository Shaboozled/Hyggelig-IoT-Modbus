#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

#include <Preferences.h>
#include <mqtt_client.h>
#include <sys/time.h>
#include <atomic>

#include "sparkplug_encoder.h"

#if __has_include("wifi_local.h")
#include "wifi_local.h"
#else
#include "wifi_local.example.h"
#endif

// eModbus: asynchronous Modbus TCP client
#include "ModbusClientTCP.h"
#include "ModbusMessage.h"

#include <WiFiClient.h>

// -----------------------------------------------------------------------------
// USER CONFIGURATION - EDIT THESE VALUES
// -----------------------------------------------------------------------------

// Local Wi-Fi network created by THIS gateway ESP32.
static const uint8_t AP_CHANNEL = 6; // AP+STA ultimately follows the upstream AP channel

// Wi-Fi network that provides internet access to this gateway.

// Static IP of the OTHER ESP32 (the Modbus TCP server) on this gateway's AP network.
// Configure the other ESP32 to use this address, subnet and gateway.
static const IPAddress MODBUS_SERVER_IP(192, 168, 50, 2);
static constexpr uint16_t MODBUS_PORT = 502;
static constexpr uint8_t MODBUS_UNIT_ID = 1;

// Modbus function 0x03 = Read Holding Registers.
// Protocol addresses are zero-based: address 0 is often labelled 40001 in manuals.
static constexpr uint16_t MODBUS_START_ADDRESS = 0;
static constexpr uint16_t MODBUS_REGISTER_COUNT = 2;
static constexpr uint32_t MODBUS_POLL_INTERVAL_MS = 2000;

// Register map expected from the other ESP32:
//   register 0: signed temperature in tenths of a degree C (e.g. 235 = 23.5 C)
//   register 1: unsigned humidity in tenths of a percent (e.g. 552 = 55.2 %)
//   register 2: unsigned pressure in tenths of hPa (e.g. 10132 = 1013.2 hPa)

// Mosquitto broker reachable through the UPLINK Wi-Fi network.
// Example for a broker on your LAN: host = PC/server LAN IP, port = 1883.
// Example for an internet-hosted broker: use its DNS name and normally TLS port 8883.
static constexpr char MQTT_URI[] = "mqtts://gruppe7.vps.webdock.cloud:8883";
static constexpr char MQTT_CLIENT_ID[] = "esp32-01-sparkplug";
static constexpr char NODE_BIRTH[] = "spBv1.0/Group7/NBIRTH/esp32-01";
static constexpr char NODE_DATA[] = "spBv1.0/Group7/NDATA/esp32-01";
static constexpr char NODE_DEATH[] = "spBv1.0/Group7/NDEATH/esp32-01";
static constexpr char DEVICE_BIRTH[] = "spBv1.0/Group7/DBIRTH/esp32-01/esp32-02";
static constexpr char DEVICE_DATA[] = "spBv1.0/Group7/DDATA/esp32-01/esp32-02";
static constexpr char NODE_COMMAND[] = "spBv1.0/Group7/NCMD/esp32-01";
// -----------------------------------------------------------------------------
// MQ-135 GAS SENSOR (analog output wired directly to THIS gateway ESP32)
// -----------------------------------------------------------------------------

// MUST be an ADC1 pin (GPIO32-39). ADC2 pins do not work while Wi-Fi is active.
// GPIO34 is input-only, which is ideal here.
static constexpr uint8_t MQ135_PIN = 34;

// The MQ-135 module's AO pin can swing up to its supply voltage (5 V), but the
// ESP32 ADC tolerates 3.3 V max. Use a voltage divider between AO and the pin.
// Ratio = (R1 + R2) / R2 where R1 is AO->pin and R2 is pin->GND.
// Two equal resistors (e.g. 10k + 10k) give a ratio of 2.0 (5 V -> 2.5 V).
static constexpr float MQ135_DIVIDER_RATIO = 2.0f;

// Voltage the module is powered from (in millivolts).
static constexpr float MQ135_SUPPLY_MV = 5000.0f;

// Load resistor on the module in kOhm. Most modules use 1k-20k; check your board.
static constexpr float MQ135_RL_KOHM = 10.0f;

// Sensor resistance in clean air (kOhm). 0 = not calibrated yet.
// With 0, the ppm estimate is published as null, but the raw values are still sent.
// To calibrate: let the sensor warm up (ideally 24 h+ the first time) in fresh air,
// read the "R0 candidate" printed on Serial, and paste that value here.
static constexpr float MQ135_R0_KOHM = 0.0f;

// CO2 curve constants (commonly used for the MQ-135): ppm = A * (Rs/R0)^(-B)
static constexpr float MQ135_CURVE_A = 116.6020682f;
static constexpr float MQ135_CURVE_B = 2.769034857f;
static constexpr float FRESH_AIR_CO2_PPM = 400.0f; // used only for the R0 suggestion

static constexpr uint32_t MQ135_WARMUP_MS = 120000;   // no publishing until warmed up
static constexpr uint32_t GAS_SAMPLE_INTERVAL_MS = 5000;
static constexpr uint8_t GAS_ADC_SAMPLES = 16;         // averaged per reading

// 0 = plain MQTT (suitable for an isolated local-network test only).
// 1 = MQTT over TLS (recommended for a broker reached over the public internet).

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

static constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

// -----------------------------------------------------------------------------
// NETWORK AND CLIENT OBJECTS
// -----------------------------------------------------------------------------

static const IPAddress AP_IP(192, 168, 50, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

WiFiClient modbusTcpSocket;
ModbusClientTCP modbusClient(modbusTcpSocket, MODBUS_SERVER_IP, MODBUS_PORT);

static esp_mqtt_client_handle_t mqttClient = nullptr;
static Preferences mqttPreferences;
static std::atomic<bool> mqttConnected{false};
static std::atomic<bool> mqttReconnect{false};
static std::atomic<bool> birthPending{false};
static uint8_t bdSeq = 255;
static uint8_t sequence = 0;
static bool deviceBorn = false;
static bool sessionRecorded = false;
static sparkplug::Buffer will;

static uint64_t utc_milliseconds() {
  timeval now{};
  gettimeofday(&now, nullptr);
  if (now.tv_sec < 1700000000) return 0;
  return uint64_t(now.tv_sec) * 1000 + now.tv_usec / 1000;
}
// -----------------------------------------------------------------------------
// SHARED DATA (Modbus callback runs on the library's worker task)
// -----------------------------------------------------------------------------

struct SensorData {
  int16_t temperatureTenths;
  uint16_t humidityTenths;
  uint64_t observedAt;
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

// MQ-135 data is only touched from loop(), so no critical section is needed.
struct GasData {
  uint64_t observedAt;
  float sensorMv;  // analog output voltage at the module (after undoing the divider)
  float rsKohm;    // computed sensor resistance, NAN if not computable
  float co2Ppm;    // estimated CO2 ppm, NAN if R0 isn't calibrated
};

static GasData latestGas = {};
static bool gasReady = false;
static uint32_t lastGasSampleMs = 0;

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
  if (message.getFunctionCode() != 0x03 || message.size() != 7 || message[2] != 4) {
    Serial.println("Modbus response had an unexpected function or length.");
    portENTER_CRITICAL(&dataMux);
    modbusRequestPending = false;
    portEXIT_CRITICAL(&dataMux);
    return;
  }

  int16_t temperatureRaw = 0;
  uint16_t humidityRaw = 0;

  // Data begins at byte offset 3 in the Modbus response message.
  message.get(3, temperatureRaw);
  message.get(5, humidityRaw);
  const uint64_t observedAt = utc_milliseconds();

  portENTER_CRITICAL(&dataMux);
  latestData.temperatureTenths = temperatureRaw;
  latestData.humidityTenths = humidityRaw;
  latestData.observedAt = observedAt;
  latestData.sequence++;
  dataReady = true;
  modbusRequestPending = false;
  portEXIT_CRITICAL(&dataMux);

  Serial.printf("Modbus OK: T=%.1f C, RH=%.1f %%\n", temperatureRaw / 10.0f, humidityRaw / 10.0f);
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

static void on_mqtt(void*, esp_event_base_t, int32_t id, void* data) {
  auto event = static_cast<esp_mqtt_event_handle_t>(data);
  if (id == MQTT_EVENT_CONNECTED) {
    mqttConnected = true;
    birthPending = true;
    esp_mqtt_client_subscribe(event->client, NODE_COMMAND, 1);
  } else if (id == MQTT_EVENT_DISCONNECTED || id == MQTT_EVENT_ERROR) {
    mqttConnected = false;
    mqttReconnect = true;
  } else if (id == MQTT_EVENT_DATA &&
             event->topic_len == sizeof(NODE_COMMAND) - 1 &&
             event->data_len == event->total_data_len &&
             !memcmp(event->topic, NODE_COMMAND, sizeof(NODE_COMMAND) - 1) &&
             sparkplug::is_rebirth(reinterpret_cast<const uint8_t*>(event->data), event->data_len)) {
    birthPending = true;
  }
}

void maintainMqtt() {
  if (mqttClient && mqttReconnect.exchange(false)) {
    esp_mqtt_client_stop(mqttClient);
    esp_mqtt_client_destroy(mqttClient);
    mqttClient = nullptr;
  }
  if (mqttConnected && !sessionRecorded) {
    mqttPreferences.putUChar("bd-seq", bdSeq);
    sessionRecorded = true;
  }
  if (mqttClient || WiFi.status() != WL_CONNECTED || !utc_milliseconds()) return;

  const uint32_t now = millis();
  if (now - lastMqttAttemptMs < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttAttemptMs = now;
  bdSeq = mqttPreferences.getUChar("bd-seq", 255) + 1;
  sessionRecorded = false;

  const auto timestamp = utc_milliseconds();
  will = sparkplug::payload(timestamp, -1);
  sparkplug::add_bd_seq(will, timestamp, bdSeq);
  esp_mqtt_client_config_t config{};
  config.uri = MQTT_URI;
  config.client_id = MQTT_CLIENT_ID;
  config.cert_pem = MQTT_CA_CERT;
  config.lwt_topic = NODE_DEATH;
  config.lwt_msg = reinterpret_cast<const char*>(will.data);
  config.lwt_msg_len = will.size;
  config.lwt_qos = 1;
  config.keepalive = 30;
  config.disable_auto_reconnect = true;
  config.network_timeout_ms = 5000;
  mqttReconnect = false;
  mqttClient = esp_mqtt_client_init(&config);
  if (!mqttClient) return;
  if (esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_ANY, on_mqtt, nullptr) != ESP_OK ||
      esp_mqtt_client_start(mqttClient) != ESP_OK) {
    esp_mqtt_client_destroy(mqttClient);
    mqttClient = nullptr;
  }
}

static bool publish_sparkplug(const char* topic, const sparkplug::Buffer& message) {
  if (message.valid && mqttConnected &&
      esp_mqtt_client_publish(mqttClient, topic,
          reinterpret_cast<const char*>(message.data), message.size, 0, false) >= 0) {
    ++sequence;
    return true;
  }
  return false;
}

static bool publish_node_birth() {
  const auto time = utc_milliseconds();
  if (!time) return false;
  sequence = 0;
  deviceBorn = false;
  auto message = sparkplug::payload(time, sequence);
  sparkplug::add_bd_seq(message, time, bdSeq);
  sparkplug::add_rebirth_control(message, time);
  const bool available = latestGas.observedAt != 0;
  sparkplug::add_double(message, "sensor_mv", 1, time, available ? latestGas.sensorMv : NAN);
  sparkplug::add_double(message, "rs_kohm", 2, time, available ? latestGas.rsKohm : NAN);
  sparkplug::add_double(message, "co2_ppm_est", 3, time, available ? latestGas.co2Ppm : NAN);
  if (!publish_sparkplug(NODE_BIRTH, message)) return false;
  Serial.println("Sparkplug NBIRTH sent");
  return true;
}

void publishLatestData() {
  SensorData snapshot;
  bool ready;
  portENTER_CRITICAL(&dataMux);
  snapshot = latestData;
  ready = dataReady;
  portEXIT_CRITICAL(&dataMux);
  if (!snapshot.observedAt) return;
  const auto time = utc_milliseconds();
  if (!time) return;
  if (!deviceBorn) {
    auto birth = sparkplug::payload(time, sequence);
    sparkplug::add_double(birth, "temperature_c", 4, snapshot.observedAt, snapshot.temperatureTenths / 10.0);
    sparkplug::add_double(birth, "humidity_pct", 5, snapshot.observedAt, snapshot.humidityTenths / 10.0);
    if (!publish_sparkplug(DEVICE_BIRTH, birth)) return;
    deviceBorn = true;
    Serial.println("Sparkplug DBIRTH sent");
  }
  if (!ready) return;
  auto message = sparkplug::payload(time, sequence);
  sparkplug::add_double(message, nullptr, 4, snapshot.observedAt, snapshot.temperatureTenths / 10.0);
  sparkplug::add_double(message, nullptr, 5, snapshot.observedAt, snapshot.humidityTenths / 10.0);
  if (publish_sparkplug(DEVICE_DATA, message)) {
    portENTER_CRITICAL(&dataMux);
    if (latestData.sequence == snapshot.sequence) dataReady = false;
    portEXIT_CRITICAL(&dataMux);
  }
}

void publishGasData() {
  if (!gasReady || !latestGas.observedAt) return;
  const auto time = utc_milliseconds();
  if (!time) return;
  auto message = sparkplug::payload(time, sequence);
  sparkplug::add_double(message, nullptr, 1, latestGas.observedAt, latestGas.sensorMv);
  sparkplug::add_double(message, nullptr, 2, latestGas.observedAt, latestGas.rsKohm);
  sparkplug::add_double(message, nullptr, 3, latestGas.observedAt, latestGas.co2Ppm);
  if (publish_sparkplug(NODE_DATA, message)) gasReady = false;
}

// -----------------------------------------------------------------------------
// MQ-135 SAMPLING
// -----------------------------------------------------------------------------

void sampleGasSensor() {
  const uint32_t now = millis();
  if (now - lastGasSampleMs < GAS_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastGasSampleMs = now;

  // The heater needs time to stabilise; readings before that are meaningless.
  if (now < MQ135_WARMUP_MS) {
    return;
  }

  // Average several calibrated readings to smooth out ADC and Wi-Fi noise.
  uint32_t sumMv = 0;
  for (uint8_t i = 0; i < GAS_ADC_SAMPLES; i++) {
    sumMv += analogReadMilliVolts(MQ135_PIN);
  }
  const float pinMv = static_cast<float>(sumMv) / GAS_ADC_SAMPLES;
  const float sensorMv = pinMv * MQ135_DIVIDER_RATIO;

  float rsKohm = NAN;
  float co2Ppm = NAN;

  // The module's load resistor RL and the sensor Rs form a voltage divider:
  //   Vout = Vcc * RL / (Rs + RL)  ->  Rs = RL * (Vcc - Vout) / Vout
  if (sensorMv > 1.0f && sensorMv < MQ135_SUPPLY_MV) {
    rsKohm = MQ135_RL_KOHM * (MQ135_SUPPLY_MV - sensorMv) / sensorMv;

    // Calibration help: which R0 would make the current reading equal fresh air?
    const float r0Candidate =
        rsKohm * powf(FRESH_AIR_CO2_PPM / MQ135_CURVE_A, 1.0f / MQ135_CURVE_B);

    if (MQ135_R0_KOHM > 0.0f) {
      co2Ppm = MQ135_CURVE_A * powf(rsKohm / MQ135_R0_KOHM, -MQ135_CURVE_B);
    }

    Serial.printf("MQ-135: pin=%.0f mV, sensor=%.0f mV, Rs=%.2f kOhm, "
                  "R0 candidate (if this is fresh air)=%.2f kOhm\n",
                  pinMv, sensorMv, rsKohm, r0Candidate);
  } else {
    Serial.printf("MQ-135: reading out of range (pin=%.0f mV). Check wiring/divider.\n", pinMv);
  }

  latestGas.observedAt = utc_milliseconds();
  latestGas.sensorMv = sensorMv;
  latestGas.rsKohm = rsKohm;
  latestGas.co2Ppm = co2Ppm;
  gasReady = true;
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

  // MQ-135 analog input (ADC1). 11 dB attenuation gives roughly a 0-3.1 V range.
  pinMode(MQ135_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  Serial.printf("MQ-135 on GPIO%u, warming up for %lu s before publishing.\n",
                MQ135_PIN, static_cast<unsigned long>(MQ135_WARMUP_MS / 1000));

  startWifi();

  mqttPreferences.begin("sparkplug", false);
  // The gateway waits for the UTC clock before opening the Sparkplug session.
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

  pollModbusServer();
  sampleGasSensor();

  if (mqttConnected) {
    if (birthPending.exchange(false) && !publish_node_birth()) birthPending = true;
    if (!birthPending) {
      publishLatestData();
      publishGasData();
    }
  }
  delay(5);
}

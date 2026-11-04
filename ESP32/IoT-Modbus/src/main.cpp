
#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "sensors.h"
#include "outputs.h"
#include "modbus_server.h"

unsigned long lastSensorRead = 0;

const unsigned long SENSOR_INTERVAL = 2000;

void connectWiFi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");

    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
}

void setup()
{
    Serial.begin(115200);

    initOutputs();
    initSensors();

    connectWiFi();

    initModbus();
}

void loop()
{
    unsigned long currentMillis = millis();

    if (currentMillis - lastSensorRead >= SENSOR_INTERVAL)
    {
        lastSensorRead = currentMillis;

        readSensors();

        updateModbusRegisters();
    }

    processModbusCommands();

    delay(10);
}

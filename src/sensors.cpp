
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "config.h"
#include "sensors.h"

#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);
BH1750 lightSensor;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

float temperature = 0;
float humidity = 0;
float light = 0;
float extraTemperature = 0;

void initSensors()
{
    dht.begin();

    Wire.begin(SDA_PIN, SCL_PIN);
    lightSensor.begin();

    ds18b20.begin();

    Serial.println("Sensors initialized");
}

void readSensors()
{
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t))
        temperature = t;

    if (!isnan(h))
        humidity = h;

    light = lightSensor.readLightLevel();

    ds18b20.requestTemperatures();
    float dsTemp = ds18b20.getTempCByIndex(0);

    if (dsTemp != DEVICE_DISCONNECTED_C)
        extraTemperature = dsTemp;

    Serial.println("----- SENSOR DATA -----");

    Serial.print("Temperature: ");
    Serial.println(temperature);

    Serial.print("Humidity: ");
    Serial.println(humidity);

    Serial.print("Light: ");
    Serial.println(light);

    Serial.print("Extra temperature: ");
    Serial.println(extraTemperature);
}

float getTemperature()
{
    return temperature;
}

float getHumidity()
{
    return humidity;
}

float getLight()
{
    return light;
}

float getExtraTemperature()
{
    return extraTemperature;
}

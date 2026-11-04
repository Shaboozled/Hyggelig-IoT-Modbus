
#include <Arduino.h>
#include "config.h"
#include "outputs.h"

bool fanStatus = false;
bool alarmStatus = false;

void initOutputs()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

void setFan(bool state)
{
    fanStatus = state;
    digitalWrite(FAN_PIN, state ? HIGH : LOW);
}

void setAlarm(bool state)
{
    alarmStatus = state;
    digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

void setLED(bool state)
{
    digitalWrite(LED_PIN, state ? HIGH : LOW);
}

bool getFanStatus()
{
    return fanStatus;
}

bool getAlarmStatus()
{
    return alarmStatus;
}

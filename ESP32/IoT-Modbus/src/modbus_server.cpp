
#include <Arduino.h>
#include <WiFi.h>

#include "ModbusServerWiFi.h"

#include "config.h"
#include "sensors.h"
#include "outputs.h"
#include "modbus_registers.h"

ModbusServerWiFi modbusServer;

uint16_t inputRegisters[INPUT_REGISTER_COUNT] = {0};
uint16_t holdingRegisters[HOLDING_REGISTER_COUNT] = {0};

// Read Input Registers (Function Code 04)
ModbusMessage readInputRegisters(ModbusMessage request)
{
    uint16_t address;
    uint16_t count;

    request.get(2, address);
    request.get(4, count);

    if (count == 0 ||
        address + count > INPUT_REGISTER_COUNT)
    {
        ModbusMessage error;
        error.setError(
            request.getServerID(),
            request.getFunctionCode(),
            ILLEGAL_DATA_ADDRESS
        );
        return error;
    }

    ModbusMessage response(
        request.getServerID(),
        request.getFunctionCode()
    );

    response.add((uint8_t)(count * 2));

    for (uint16_t i = 0; i < count; i++)
    {
        response.add(inputRegisters[address + i]);
    }

    return response;
}

// Read Holding Registers (Function Code 03)
ModbusMessage readHoldingRegisters(ModbusMessage request)
{
    uint16_t address;
    uint16_t count;

    request.get(2, address);
    request.get(4, count);

    if (count == 0 ||
        address + count > HOLDING_REGISTER_COUNT)
    {
        ModbusMessage error;
        error.setError(
            request.getServerID(),
            request.getFunctionCode(),
            ILLEGAL_DATA_ADDRESS
        );
        return error;
    }

    ModbusMessage response(
        request.getServerID(),
        request.getFunctionCode()
    );

    response.add((uint8_t)(count * 2));

    for (uint16_t i = 0; i < count; i++)
    {
        response.add(holdingRegisters[address + i]);
    }

    return response;
}

// Write Single Holding Register (Function Code 06)
ModbusMessage writeHoldingRegister(ModbusMessage request)
{
    uint16_t address;
    uint16_t value;

    request.get(2, address);
    request.get(4, value);

    if (address >= HOLDING_REGISTER_COUNT)
    {
        ModbusMessage error;
        error.setError(
            request.getServerID(),
            request.getFunctionCode(),
            ILLEGAL_DATA_ADDRESS
        );
        return error;
    }

    if (address > REG_LED_COMMAND || value > 1)
    {
        ModbusMessage error;
        error.setError(
            request.getServerID(),
            request.getFunctionCode(),
            ILLEGAL_DATA_VALUE
        );
        return error;
    }

    holdingRegisters[address] = value;

    return ECHO_RESPONSE;
}

void initModbus()
{
    modbusServer.registerWorker(
        1, READ_INPUT_REGISTER, readInputRegisters
    );

    modbusServer.registerWorker(
        1, READ_HOLD_REGISTER, readHoldingRegisters
    );

    modbusServer.registerWorker(
        1, WRITE_HOLD_REGISTER, writeHoldingRegister
    );

    if (!modbusServer.start(502, 2, 10000))
    {
        Serial.println("Modbus server failed!");
    }
    else
    {
        Serial.println("Modbus TCP Server started!");
    }
}

void updateModbusRegisters()
{
    inputRegisters[REG_TEMPERATURE] =
        (uint16_t)(getTemperature() * 10);

    inputRegisters[REG_HUMIDITY] =
        (uint16_t)(getHumidity() * 10);

    inputRegisters[REG_LIGHT] =
        (uint16_t)constrain(getLight(), 0, 65535);

    inputRegisters[REG_EXTRA_TEMP] =
        (uint16_t)(getExtraTemperature() * 10);

    holdingRegisters[REG_FAN_STATUS] =
        getFanStatus();

    holdingRegisters[REG_ALARM_STATUS] =
        getAlarmStatus();

    holdingRegisters[REG_SYSTEM_STATUS] = 1;
}

void processModbusCommands()
{
    setFan(holdingRegisters[REG_FAN_COMMAND] == 1);

    setAlarm(holdingRegisters[REG_ALARM_COMMAND] == 1);

    setLED(holdingRegisters[REG_LED_COMMAND] == 1);
}

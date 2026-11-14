/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify this software
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include <stdint.h>

#include "platform.h"

#include "common/maths.h"
#include "common/utils.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "drivers/ina2xx.h"
#include "sensors/battery.h"
#if defined(USE_DRONECAN)
#include "sensors/battery_sensor_dronecan.h"
#endif

#define BATTERY2_READ_INTERVAL_US      100000
#define BATTERY2_MAH_DIVISOR           360000000LL
#define BATTERY2_VOLTAGE_HYSTERESIS    10
#define BATTERY2_CAN_TIMEOUT_MS        1500

PG_REGISTER_WITH_RESET_TEMPLATE(battery2Config_t, battery2Config, PG_BATTERY2_CONFIG, 1);

PG_RESET_TEMPLATE(battery2Config_t, battery2Config,
    .voltageScale = 1000,
    .voltageOffset = 0,
    .currentScale = 2,
    .currentOffset = 0,
    .cells = 0,
    .cellDetect = 430,
    .cellWarning = 350,
    .cellCritical = 330,
    .voltageEnabled = 1,
    .voltageSensor = BATTERY2_SENSOR_I2C,
    .currentEnabled = 1,
    .currentSensor = BATTERY2_SENSOR_I2C,
    .canBatteryId = 1
);

static uint16_t battery2Voltage;
static int16_t battery2Amperage;
static uint32_t battery2MAhDrawn;
static uint8_t battery2Cells;
static battery2State_e battery2State = BATTERY2_UNAVAILABLE;
static bool battery2Present;
static bool battery2Ambiguous;
static bool battery2HasPreviousSample;
static int64_t battery2MAhAccumulator;
static timeUs_t battery2PollElapsed;
static uint16_t battery2CanVoltage;
static int16_t battery2CanAmperage;
static uint32_t battery2CanLastUpdateMs;
static bool battery2CanSampleReceived;

#if defined(USE_BATTERY2_INA2XX)
static ina2xxDev_t battery2Ina;

static bool battery2DetectSensor(void)
{
    static const uint8_t addresses[] = {0x45, 0x44, 0x41};
    uint8_t detectedCount = 0;

    for (uint8_t index = 0; index < ARRAYLEN(addresses); index++) {
        ina2xxDev_t candidate = {0};
        if (!ina2xxInit(&candidate, BATTERY2_I2C_BUS, addresses[index], index)) {
            continue;
        }

        detectedCount++;
        if (detectedCount == 1) {
            battery2Ina = candidate;
        } else {
            ina2xxDeInit(&candidate);
            ina2xxDeInit(&battery2Ina);
            battery2Ambiguous = true;
            return false;
        }
    }

    return detectedCount == 1;
}
#endif

void battery2Init(void)
{
    battery2Voltage = 0;
    battery2Amperage = 0;
    battery2MAhDrawn = 0;
    battery2Cells = 0;
    battery2State = BATTERY2_UNAVAILABLE;
    battery2Present = false;
    battery2Ambiguous = false;
    battery2HasPreviousSample = false;
    battery2MAhAccumulator = 0;
    battery2PollElapsed = 0;

    // Older battery2 configs used a 1000-based multiplier for current.
    // Convert the old unity default to the Matek 200 uOhm shunt scale
    // (0.2 mV/A, represented in 0.1 mV/A units).
    if (battery2Config()->currentScale >= 100) {
        battery2ConfigMutable()->currentScale = 2;
        battery2ConfigMutable()->currentOffset = 0;
    }

#if defined(USE_BATTERY2_INA2XX)
    if ((battery2Config()->voltageEnabled && battery2Config()->voltageSensor == BATTERY2_SENSOR_I2C)
        || (battery2Config()->currentEnabled && battery2Config()->currentSensor == BATTERY2_SENSOR_I2C)) {
        battery2DetectSensor();
    }
#endif
}

void battery2ReceiveCanInfo(uint8_t batteryId, uint16_t voltage, int16_t amperage)
{
#if defined(USE_DRONECAN)
    if (batteryId == battery2Config()->canBatteryId) {
        battery2CanVoltage = voltage;
        battery2CanAmperage = amperage;
        battery2CanLastUpdateMs = millis();
        battery2CanSampleReceived = true;
    }
#else
    UNUSED(batteryId);
    UNUSED(voltage);
    UNUSED(amperage);
#endif
}

static uint16_t battery2ApplyVoltageCalibration(uint16_t rawVoltage)
{
    const int32_t calibrated = ((int32_t)rawVoltage * battery2Config()->voltageScale / 1000)
        + battery2Config()->voltageOffset;
    return (uint16_t)constrain(calibrated, 0, UINT16_MAX);
}

static int16_t battery2ApplyCurrentCalibration(int32_t shuntMicroVolts)
{
    const int32_t calibrated = (shuntMicroVolts - (int32_t)battery2Config()->currentOffset * 1000)
        / MAX(1, battery2Config()->currentScale);
    return (int16_t)constrain(calibrated, INT16_MIN, INT16_MAX);
}

static uint8_t battery2DetectCells(uint16_t voltage)
{
    if (battery2Config()->cells > 0) {
        return MIN(battery2Config()->cells, 12);
    }

    const uint16_t cellDetect = battery2Config()->cellDetect;
    if (cellDetect == 0) {
        return 0;
    }

    uint8_t cells = (voltage / cellDetect) + 1;
    if (cells == 7 || cells == 9 || cells == 11) {
        cells++;
    }
    return MIN(cells, 12);
}

static void battery2UpdateVoltageState(void)
{
    if (!battery2Present) {
        battery2Cells = 0;
        battery2State = BATTERY2_UNAVAILABLE;
        return;
    }

    battery2Cells = battery2DetectCells(battery2Voltage);
    if (battery2Cells == 0 || battery2Config()->cellWarning == 0 || battery2Config()->cellCritical == 0) {
        battery2State = BATTERY2_UNAVAILABLE;
        return;
    }

    const uint16_t warningVoltage = battery2Cells * battery2Config()->cellWarning;
    const uint16_t criticalVoltage = battery2Cells * battery2Config()->cellCritical;

    switch (battery2State) {
    case BATTERY2_UNAVAILABLE:
        battery2State = battery2Voltage <= criticalVoltage ? BATTERY2_CRITICAL
            : battery2Voltage <= warningVoltage ? BATTERY2_WARNING : BATTERY2_OK;
        break;
    case BATTERY2_OK:
        if (battery2Voltage <= criticalVoltage) {
            battery2State = BATTERY2_CRITICAL;
        } else if (battery2Voltage <= warningVoltage) {
            battery2State = BATTERY2_WARNING;
        }
        break;
    case BATTERY2_WARNING:
        if (battery2Voltage <= criticalVoltage) {
            battery2State = BATTERY2_CRITICAL;
        } else if (battery2Voltage > warningVoltage + BATTERY2_VOLTAGE_HYSTERESIS) {
            battery2State = BATTERY2_OK;
        }
        break;
    case BATTERY2_CRITICAL:
        if (battery2Voltage > criticalVoltage + BATTERY2_VOLTAGE_HYSTERESIS) {
            battery2State = BATTERY2_WARNING;
        }
        break;
    }
}

void battery2Update(timeUs_t timeDelta)
{
    battery2PollElapsed += timeDelta;
    if (battery2PollElapsed < BATTERY2_READ_INTERVAL_US) {
        return;
    }

    const timeUs_t sampleTime = battery2PollElapsed;
    battery2PollElapsed = 0;

    const battery2Config_t *config = battery2Config();
    bool voltageAvailable = !config->voltageEnabled;
    bool currentAvailable = !config->currentEnabled;
    uint16_t voltage = 0;
    int16_t amperage = 0;
    bool voltageIsCan = false;
    const bool canFresh = battery2CanSampleReceived
        && (millis() - battery2CanLastUpdateMs) <= BATTERY2_CAN_TIMEOUT_MS;

#if defined(USE_BATTERY2_INA2XX)
    if (battery2Ina.busDev && !battery2Ambiguous) {
        uint16_t rawVoltage;
        int32_t rawShuntVoltage;
        if (config->voltageEnabled && config->voltageSensor == BATTERY2_SENSOR_I2C
            && ina2xxReadBusVoltage(&battery2Ina, &rawVoltage)) {
            voltage = rawVoltage;
            voltageAvailable = true;
        }
        if (config->currentEnabled && config->currentSensor == BATTERY2_SENSOR_I2C
            && ina2xxReadShuntVoltage(&battery2Ina, &rawShuntVoltage)) {
            amperage = battery2ApplyCurrentCalibration(rawShuntVoltage);
            currentAvailable = true;
        }
    }
#endif

#if defined(USE_DRONECAN)
    if (canFresh && config->voltageEnabled && config->voltageSensor == BATTERY2_SENSOR_CAN) {
        voltage = battery2CanVoltage;
        voltageAvailable = true;
        voltageIsCan = true;
    }
    if (canFresh && config->currentEnabled && config->currentSensor == BATTERY2_SENSOR_CAN) {
        amperage = battery2CanAmperage;
        currentAvailable = true;
    }
#endif

    battery2Present = (config->voltageEnabled || config->currentEnabled)
        && voltageAvailable && currentAvailable;
    battery2Voltage = config->voltageEnabled && voltageAvailable
        ? (voltageIsCan ? voltage : battery2ApplyVoltageCalibration(voltage)) : 0;
    battery2Amperage = config->currentEnabled && currentAvailable
        ? amperage : 0;

    if (!battery2Present) {
        battery2HasPreviousSample = false;
        battery2State = BATTERY2_UNAVAILABLE;
        battery2Cells = 0;
        return;
    }

    // Do not count reverse current as consumed capacity.
    if (battery2HasPreviousSample) {
        battery2MAhAccumulator += (int64_t)MAX(0, battery2Amperage) * sampleTime;
    }
    battery2HasPreviousSample = true;
    battery2MAhDrawn = battery2MAhAccumulator / BATTERY2_MAH_DIVISOR;
    if (config->voltageEnabled) {
        battery2UpdateVoltageState();
    } else {
        battery2State = BATTERY2_UNAVAILABLE;
        battery2Cells = 0;
    }
}

bool isBattery2Supported(void)
{
#if defined(USE_BATTERY2_INA2XX) || defined(USE_DRONECAN)
    return true;
#else
    return false;
#endif
}

bool isBattery2Present(void)
{
    return battery2Present && !battery2Ambiguous;
}

battery2State_e getBattery2State(void)
{
    return battery2State;
}

uint16_t getBattery2Voltage(void)
{
    return isBattery2Present() ? battery2Voltage : 0;
}

int16_t getBattery2Amperage(void)
{
    return isBattery2Present() ? battery2Amperage : 0;
}

uint32_t getBattery2MahDrawn(void)
{
    return battery2MAhDrawn;
}

uint8_t getBattery2CellCount(void)
{
    return isBattery2Present() ? battery2Cells : 0;
}

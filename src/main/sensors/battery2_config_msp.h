/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "common/streambuf.h"
#include "sensors/battery_config_structs.h"

// Parse into a temporary value. Invalid or truncated packets never change RAM.
static inline bool battery2ConfigDecode(sbuf_t *src, const battery2Config_t *previous, battery2Config_t *result)
{
    const int dataSize = sbufBytesRemaining(src);
    if (dataSize != 15 && dataSize != 20 && dataSize != 21) {
        return false;
    }
    battery2Config_t config = {
        .voltageScale = sbufReadU16(src),
        .voltageOffset = (int16_t)sbufReadU16(src),
        .currentScale = sbufReadU16(src),
        .currentOffset = (int16_t)sbufReadU16(src),
        .cells = sbufReadU8(src),
        .cellDetect = sbufReadU16(src),
        .cellWarning = sbufReadU16(src),
        .cellCritical = sbufReadU16(src),
        .voltageEnabled = previous->voltageEnabled,
        .voltageSensor = previous->voltageSensor,
        .currentEnabled = previous->currentEnabled,
        .currentSensor = previous->currentSensor,
        .canBatteryId = previous->canBatteryId,
        .currentI2cAddress = previous->currentI2cAddress,
    };

    if (dataSize >= 20) {
        config.voltageEnabled = sbufReadU8(src);
        config.voltageSensor = sbufReadU8(src);
        config.currentEnabled = sbufReadU8(src);
        config.currentSensor = sbufReadU8(src);
        config.canBatteryId = sbufReadU8(src);
    }
    if (dataSize == 21) {
        config.currentI2cAddress = sbufReadU8(src);
    }

    if (config.voltageScale == 0 || config.currentScale == 0 || config.cells > 12
        || config.voltageEnabled > 1 || config.currentEnabled > 1
        || config.voltageSensor > BATTERY2_SENSOR_ADC || config.currentSensor > BATTERY2_SENSOR_ADC
        || config.currentI2cAddress < 0x40 || config.currentI2cAddress > 0x4F
#if !defined(USE_BATTERY2_ADC)
        || config.voltageSensor == BATTERY2_SENSOR_ADC || config.currentSensor == BATTERY2_SENSOR_ADC
#endif
        || config.cellDetect < 250 || config.cellDetect > 500
        || config.cellCritical < 250 || config.cellCritical > 450
        || config.cellWarning < 250 || config.cellWarning > 450
        || config.cellCritical >= config.cellWarning) {
        return false;
    }

    *result = config;
    return true;
}

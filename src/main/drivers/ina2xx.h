/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify this software
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/bus.h"

typedef enum {
    INA2XX_DEVICE_NONE,
    INA2XX_DEVICE_228,
    INA2XX_DEVICE_238,
} ina2xxDeviceType_e;

typedef struct ina2xxDev_s {
    busDevice_t *busDev;
    ina2xxDeviceType_e type;
} ina2xxDev_t;

bool ina2xxInit(ina2xxDev_t *dev, I2CDevice i2cBus, uint8_t i2cAddress, uint8_t index);
void ina2xxDeInit(ina2xxDev_t *dev);
bool ina2xxReadBusVoltage(ina2xxDev_t *dev, uint16_t *centiVolts);
bool ina2xxReadCurrent(ina2xxDev_t *dev, int16_t *centiAmps);


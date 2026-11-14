/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify this software
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_BATTERY2_INA2XX)

#include "drivers/ina2xx.h"

#define INA2XX_REG_VBUS             0x05
#define INA2XX_REG_VSHUNT           0x04
#define INA2XX_REG_CURRENT          0x07
#define INA2XX_REG_CONFIG           0x00
#define INA2XX_REG_SHUNT_CAL        0x02
#define INA2XX_REG_MANUFACTURER_ID  0x3E
#define INA2XX_REG_DEVICE_ID        0x3F

#define INA2XX_MANUFACTURER_ID      0x5449
#define INA2XX_CONFIG_RESET         0x8000
#define INA2XX_SHUNT_CAL            1024
#define INA228_DEVICE_ID_MASK        0xFFF0
#define INA228_DEVICE_ID             0x2280
#define INA238_DEVICE_ID             0x2380

static bool ina2xxReadRegister16(ina2xxDev_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];
    if (!dev || !dev->busDev || !value || !busReadBuf(dev->busDev, reg, buffer, sizeof(buffer))) {
        return false;
    }

    *value = ((uint16_t)buffer[0] << 8) | buffer[1];
    return true;
}

static bool ina2xxReadRegister24(ina2xxDev_t *dev, uint8_t reg, uint32_t *value)
{
    uint8_t buffer[3];
    if (!dev || !dev->busDev || !value || !busReadBuf(dev->busDev, reg, buffer, sizeof(buffer))) {
        return false;
    }

    *value = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    return true;
}

static bool ina2xxWriteRegister16(ina2xxDev_t *dev, uint8_t reg, uint16_t value)
{
    if (!dev || !dev->busDev) {
        return false;
    }

    const uint8_t buffer[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return busWriteBuf(dev->busDev, reg, buffer, sizeof(buffer));
}

static bool ina2xxI2cBusIsEnabled(I2CDevice i2cBus)
{
    switch (i2cBus) {
#ifdef USE_I2C_DEVICE_1
    case I2CDEV_1:
        return true;
#endif
#ifdef USE_I2C_DEVICE_2
    case I2CDEV_2:
        return true;
#endif
#ifdef USE_I2C_DEVICE_3
    case I2CDEV_3:
        return true;
#endif
#ifdef USE_I2C_DEVICE_4
    case I2CDEV_4:
        return true;
#endif
    default:
        return false;
    }
}

static bool ina2xxDetect(ina2xxDev_t *dev)
{
    uint16_t manufacturerId;
    uint16_t deviceId;
    if (!ina2xxReadRegister16(dev, INA2XX_REG_MANUFACTURER_ID, &manufacturerId)
        || !ina2xxReadRegister16(dev, INA2XX_REG_DEVICE_ID, &deviceId)
        || manufacturerId != INA2XX_MANUFACTURER_ID) {
        return false;
    }

    if ((deviceId & INA228_DEVICE_ID_MASK) == INA228_DEVICE_ID) {
        dev->type = INA2XX_DEVICE_228;
    } else if ((deviceId & INA228_DEVICE_ID_MASK) == INA238_DEVICE_ID) {
        dev->type = INA2XX_DEVICE_238;
    } else {
        return false;
    }

    // Reset to a known continuous-measurement configuration, then calibrate
    // the current register for the module's specified 200 uOhm shunt and
    // 204.8 A range (as used by Matek's INA2XX module configuration).
    return ina2xxWriteRegister16(dev, INA2XX_REG_CONFIG, INA2XX_CONFIG_RESET)
        && ina2xxWriteRegister16(dev, INA2XX_REG_CONFIG, 0)
        && ina2xxWriteRegister16(dev, INA2XX_REG_SHUNT_CAL, INA2XX_SHUNT_CAL);
}

bool ina2xxInit(ina2xxDev_t *dev, I2CDevice i2cBus, uint8_t i2cAddress, uint8_t index)
{
    if (!dev || i2cAddress < 0x40 || i2cAddress > 0x4F || !ina2xxI2cBusIsEnabled(i2cBus)) {
        return false;
    }

    dev->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_INA2XX, index, OWNER_CURRENT_METER);
    if (!dev->busDev) {
        return false;
    }

    dev->busDev->busdev.i2c.i2cBus = i2cBus;
    dev->busDev->busdev.i2c.address = i2cAddress;
    dev->type = INA2XX_DEVICE_NONE;

    if (!ina2xxDetect(dev)) {
        ina2xxDeInit(dev);
        return false;
    }

    return true;
}

void ina2xxDeInit(ina2xxDev_t *dev)
{
    if (dev && dev->busDev) {
        busDeviceDeInit(dev->busDev);
        dev->busDev = NULL;
        dev->type = INA2XX_DEVICE_NONE;
    }
}

bool ina2xxReadBusVoltage(ina2xxDev_t *dev, uint16_t *centiVolts)
{
    if (!dev || !centiVolts) {
        return false;
    }

    uint32_t raw;
    if (dev->type == INA2XX_DEVICE_228) {
        if (!ina2xxReadRegister24(dev, INA2XX_REG_VBUS, &raw)) {
            return false;
        }

        // INA228 returns a 20-bit result in bits 23..4, at 195.3125 uV/LSB.
        const uint32_t result = raw >> 4;
        *centiVolts = (uint16_t)((result * 5 + 128) / 256);
        return true;
    }

    if (dev->type == INA2XX_DEVICE_238) {
        uint16_t raw16;
        if (!ina2xxReadRegister16(dev, INA2XX_REG_VBUS, &raw16)) {
            return false;
        }

        // INA238 bus-voltage LSB is 3.125 mV.
        *centiVolts = (uint16_t)(((uint32_t)raw16 * 3125 + 5000) / 10000);
        return true;
    }

    return false;
}

bool ina2xxReadCurrent(ina2xxDev_t *dev, int16_t *centiAmps)
{
    if (!dev || !centiAmps) {
        return false;
    }

    int64_t result;
    if (dev->type == INA2XX_DEVICE_228) {
        uint32_t raw;
        if (!ina2xxReadRegister24(dev, INA2XX_REG_CURRENT, &raw)) {
            return false;
        }

        // INA228 CURRENT is signed 20-bit data left-aligned in 24 bits.
        int32_t signedResult = raw >> 4;
        if (signedResult & 0x80000) {
            signedResult -= 0x100000;
        }
        result = ((int64_t)signedResult * 5 + (signedResult >= 0 ? 64 : -64)) / 128;
    } else if (dev->type == INA2XX_DEVICE_238) {
        uint16_t raw;
        if (!ina2xxReadRegister16(dev, INA2XX_REG_CURRENT, &raw)) {
            return false;
        }

        // INA238 current LSB is 204.8 A / 2^15 = 0.00625 A.
        const int32_t signedResult = (int16_t)raw;
        result = ((int64_t)signedResult * 5 + (signedResult >= 0 ? 4 : -4)) / 8;
    } else {
        return false;
    }

    if (result > INT16_MAX) {
        result = INT16_MAX;
    } else if (result < INT16_MIN) {
        result = INT16_MIN;
    }

    *centiAmps = (int16_t)result;
    return true;
}

bool ina2xxReadShuntVoltage(ina2xxDev_t *dev, int32_t *microVolts)
{
    if (!dev || !microVolts) {
        return false;
    }

    int64_t result;
    if (dev->type == INA2XX_DEVICE_228) {
        uint32_t raw;
        if (!ina2xxReadRegister24(dev, INA2XX_REG_VSHUNT, &raw)) {
            return false;
        }

        int32_t signedResult = raw >> 4;
        if (signedResult & 0x80000) {
            signedResult -= 0x100000;
        }
        // INA228 VSHUNT has a 312.5 nV/LSB resolution.
        result = ((int64_t)signedResult * 3125 + (signedResult >= 0 ? 5000 : -5000)) / 10000;
    } else if (dev->type == INA2XX_DEVICE_238) {
        uint16_t raw;
        if (!ina2xxReadRegister16(dev, INA2XX_REG_VSHUNT, &raw)) {
            return false;
        }

        // INA238 VSHUNT has a 5 uV/LSB resolution.
        result = (int16_t)raw * 5;
    } else {
        return false;
    }

    if (result > INT32_MAX) {
        result = INT32_MAX;
    } else if (result < INT32_MIN) {
        result = INT32_MIN;
    }
    *microVolts = (int32_t)result;
    return true;
}

#endif

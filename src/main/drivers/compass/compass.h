/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stddef.h>

#include "common/axis.h"
#include "common/vector.h"

#include "drivers/bus.h"
#include "drivers/sensor.h"

typedef enum {
    MAG_READ_ERROR = 0,
    MAG_READ_NOT_READY,
    MAG_READ_READY
} magReadStatus_e;

typedef struct magDev_s {
    busDevice_t * busDev;
    // Multi-MAG owns a persistent descriptor/context; never reset registry storage.
    bool usePreconfiguredBus;
    magReadStatus_e readStatus;
    int16_t factoryGain[XYZ_AXIS_COUNT];
    uint32_t nextReadAtUs;
    sensorMagInitFuncPtr init;  // initialize function
    sensorMagReadFuncPtr read;  // read 3 axis data function
    struct {
        bool useExternal;
        union {
            fpMat3_t externalRotation;
            sensor_align_e onBoard;
        };
    } magAlign;
    uint8_t magSensorToUse;
    int16_t magADCRaw[XYZ_AXIS_COUNT];
} magDev_t;

// A preconfigured context belongs to the caller, not to the global bus registry.
static inline busDevice_t *magBusDeviceInit(magDev_t *mag, busType_e bus, devHardwareType_e hardware)
{
    if (!mag->usePreconfiguredBus) {
        return busDeviceInit(bus, hardware, mag->magSensorToUse, OWNER_COMPASS);
    }
    busDevice_t *device = mag->busDev;
    if (!device || !device->descriptorPtr ||
        (bus != BUSTYPE_ANY && bus != BUSTYPE_I2C) ||
        device->busType != BUSTYPE_I2C || device->descriptorPtr->busType != BUSTYPE_I2C ||
        device->descriptorPtr->devicePtr != device || device->descriptorPtr->devHwType != hardware) {
        return NULL;
    }
    return device;
}

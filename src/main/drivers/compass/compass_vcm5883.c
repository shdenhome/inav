/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include "platform.h"

#ifdef USE_MAG_VCM5883

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "drivers/compass/compass_vcm5883.h"

#include "build/debug.h"

#define VCM5883_REGISTER_ADDR_CNTL1 0x0B
#define VCM5883_REGISTER_ADDR_CNTL2 0x0A
#define VCM5883_REGISTER_ADDR_CHIPID 0x0C
#define VCM5883_REGISTER_ADDR_OUTPUT_X 0x00
#define VCM5883_INITIAL_CONFIG 0b0100

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(magDev_t * mag)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        busWrite(mag->busDev, VCM5883_REGISTER_ADDR_CNTL2, 0b01000001);
        delay(30);

        uint8_t sig = 0;
        bool ack = busRead(mag->busDev, VCM5883_REGISTER_ADDR_CHIPID, &sig);

        if (ack && sig == 0x82) {
            return true;
        }
    }

    return false;
}

static bool vcm5883Init(magDev_t * mag) {
    if (mag->usePreconfiguredBus) {
        // Datasheet 5.2.2: 0x41 enables normal measurement at 200 Hz.
        if (!busWrite(mag->busDev, VCM5883_REGISTER_ADDR_CNTL2, 0x41)) {
            return false;
        }
        mag->nextReadAtUs = micros() + 10000;
    }
    return true;
}

static bool vcm5883Read(magDev_t * mag)
{
    mag->readStatus = MAG_READ_ERROR;
    uint8_t buf[6];
    if (mag->usePreconfiguredBus) {
        // VCM5883L has no DRDY register/pin. Allow two ODR periods between
        // readouts, and check mode before trusting the conversion timing.
        if ((int32_t)(micros() - mag->nextReadAtUs) < 0) {
            mag->readStatus = MAG_READ_NOT_READY;
            return false;
        }
        uint8_t mode;
        if (!busRead(mag->busDev, VCM5883_REGISTER_ADDR_CNTL2, &mode) || mode != 0x41) {
            return false;
        }
    }

    // set magData to zero for case of failed read
    if (!mag->usePreconfiguredBus) {
        mag->magADCRaw[X] = 0;
        mag->magADCRaw[Y] = 0;
        mag->magADCRaw[Z] = 0;
    }

    bool ack = busReadBuf(mag->busDev, VCM5883_REGISTER_ADDR_OUTPUT_X, buf, 6);
    if (!ack) {
        return false;
    }

    mag->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[0]);
    mag->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[2]);
    mag->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[4]);

    if (mag->usePreconfiguredBus) {
        mag->nextReadAtUs = micros() + 10000;
    }
    mag->readStatus = MAG_READ_READY;
    return true;
}

bool vcm5883Detect(magDev_t * mag)
{
    mag->busDev = magBusDeviceInit(mag, BUSTYPE_ANY, DEVHW_VCM5883);
    if (mag->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(mag)) {
        busDeviceDeInit(mag->busDev);
        return false;
    }

    mag->init = vcm5883Init;
    mag->read = vcm5883Read;

    return true;
}

#endif
/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_SENSOR
#include "sensors/multi_sensor_msp.h"
#include "sensors/multi_sensor.h"
#include "sensors/multi_acc.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

void multiSensorSerializeInfo(sbuf_t *dst)
{
    // Optional v1 tail of MSP_SENSOR_CONFIG, after its original six bytes.
    sbufWriteU8(dst, 'M');
    sbufWriteU8(dst, 'S');
    sbufWriteU8(dst, 1);
    sbufWriteU8(dst, 4 | (multiSensorImuEnabled() ? 1 : 0) | (multiSensorBaroEnabled() ? 2 : 0)); // bit2: state telemetry supported
    sbufWriteU8(dst, gyroGetImuTag(0));
    for (unsigned tag = 0; tag < MULTI_IMU_COUNT; tag++) {
        sbufWriteU8(dst, multiSensorImuEnabled() ? multiAccGetHardware(tag) : ACC_NONE);
    }
    for (unsigned slot = 0; slot < MULTI_BARO_COUNT; slot++) {
        sbufWriteU8(dst, baroGetSensorHardware(slot));
    }
    sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_BARO]);
}
#endif

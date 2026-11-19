/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#if defined(USE_MULTI_SENSOR) && defined(USE_BLACKBOX)
#include <stddef.h>
#include "blackbox/multi_sensor_log.h"
#include "blackbox/blackbox_encoding.h"
#include "sensors/multi_sensor_telemetry.h"

static int32_t ageMs(uint32_t ageUs) { return ageUs / 1000 > 65535 ? 65535 : ageUs / 1000; }

void multiSensorLogCapture(multiSensorLogState_t *state, uint32_t nowUs)
{
    multiSensorTelemetry_t snapshot;
    multiSensorCaptureTelemetry(&snapshot, nowUs);
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        const multiImuTelemetry_t *imu = &snapshot.imu[i];
        state->imuType[i] = imu->hardware; state->imuFlags[i] = imu->flags;
        state->imuGyroTrust[i] = imu->gyroTrust; state->imuAccTrust[i] = imu->accTrust;
        state->imuGyroAgeMs[i] = ageMs(imu->gyroAgeUs); state->imuAccAgeMs[i] = ageMs(imu->accAgeUs);
        for (unsigned axis = 0; axis < 3; axis++) {
            state->imuGyroMdps[i * 3 + axis] = imu->gyroMdps[axis];
            state->imuAccMg[i * 3 + axis] = imu->accMg[axis];
            state->imuRawAccMg[i * 3 + axis] = imu->rawAccMg[axis];
        }
    }
    for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
        const multiBaroTelemetry_t *baro = &snapshot.baro[i];
        state->baroType[i] = baro->hardware; state->baroFlags[i] = baro->flags;
        state->baroTrust[i] = baro->trust; state->baroAgeMs[i] = ageMs(baro->ageUs);
        state->baroPressurePa[i] = baro->pressurePa; state->baroAlignedPa[i] = baro->alignedPressurePa;
        state->baroTempCenti[i] = baro->temperatureCenti; state->baroAltitudeCm[i] = baro->altitudeCm;
    }
}

void multiSensorLogWriteMain(const multiSensorLogState_t *state, const multiSensorLogState_t *previous, unsigned groups)
{
#define WRITE_FIELD(name, index, member, group) \
    if (groups & (group)) blackboxWriteSignedVB((int32_t)((uint32_t)state->member - (previous ? (uint32_t)previous->member : 0)));
    MULTI_SENSOR_MAIN_LOG_FIELDS(WRITE_FIELD)
#undef WRITE_FIELD
}
void multiSensorLogWriteSlow(const multiSensorLogState_t *state, unsigned groups)
{
#define WRITE_FIELD(name, index, member, group) \
    if (groups & (group)) blackboxWriteSignedVB(state->member);
    MULTI_SENSOR_SLOW_LOG_FIELDS(WRITE_FIELD)
#undef WRITE_FIELD
}

#endif

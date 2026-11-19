/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_SENSOR
#include <math.h>
#include <limits.h>
#include <string.h>
#include "sensors/multi_sensor_telemetry.h"
#include "sensors/multi_acc.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"

static int32_t quantize(float value)
{
    if (!isfinite(value)) return 0;
    // The nearest representable float below INT32_MAX avoids float->int overflow.
    if (value >= 2147483520.0f) return INT32_MAX;
    if (value <= -2147483648.0f) return INT32_MIN;
    return lrintf(value);
}

static uint16_t trust(float confidence)
{
    return !isfinite(confidence) || confidence <= 0 ? 0 : confidence >= 1 ? 1000 : lrintf(confidence * 1000);
}

static bool valid(const sensorFusionMember_t *member, const sensorFusionSample_t *sample, uint32_t age, uint32_t maxAge)
{
    if (!member->usable || !sample->valid || age > maxAge || !isfinite(member->confidence)) return false;
    for (unsigned axis = 0; axis < 3; axis++) {
        if (!isfinite(sample->value[axis])) return false;
    }
    return true;
}

void multiSensorCaptureTelemetry(multiSensorTelemetry_t *state, uint32_t nowUs)
{
    memset(state, 0, sizeof(*state));
    state->timestampUs = nowUs;
    state->flags = (multiSensorImuEnabled() ? 1 : 0) | (multiSensorBaroEnabled() ? 2 : 0);
    const multiSensorConfig_t *config = multiSensorConfig();
    for (unsigned tag = 0; tag < MULTI_IMU_COUNT; tag++) {
        state->imu[tag].tag = tag;
        state->imu[tag].gyroAgeUs = state->imu[tag].accAgeUs = UINT32_MAX;
    }
    if (multiSensorImuEnabled()) {
        for (unsigned slot = 0; slot < MULTI_IMU_COUNT; slot++) {
            const unsigned tag = gyroGetImuTag(slot);
            if (tag >= MULTI_IMU_COUNT) continue;
            multiImuTelemetry_t *imu = &state->imu[tag];
            const sensorFusionMember_t *g = &multiGyroFusion.member[slot], *a = &multiAccFusion.member[slot];
            const sensorFusionSample_t *gs = &multiGyroSamples[slot], *as = &multiAccSamples[slot];
            imu->hardware = multiAccGetHardware(tag);
            imu->flags = MULTI_IMU_PRESENT;
            imu->gyroAgeUs = nowUs - gs->timestampUs;
            imu->accAgeUs = nowUs - as->timestampUs;
            imu->gyroTrust = trust(g->confidence); imu->accTrust = trust(a->confidence);
            if (valid(g, gs, imu->gyroAgeUs, (uint32_t)config->imuMaxAgeMs * 1000)) {
                imu->flags |= MULTI_IMU_GYRO_VALID;
                if (multiGyroFusion.contributorMask & (1u << slot)) imu->flags |= MULTI_IMU_GYRO_CONTRIBUTOR;
            }
            if (config->accCalibratedMask & (1u << tag)) imu->flags |= MULTI_IMU_ACC_CALIBRATED;
            if ((imu->flags & MULTI_IMU_ACC_CALIBRATED) && valid(a, as, imu->accAgeUs, (uint32_t)config->imuMaxAgeMs * 1000)) {
                imu->flags |= MULTI_IMU_ACC_VALID;
                if (multiAccFusion.contributorMask & (1u << slot)) imu->flags |= MULTI_IMU_ACC_CONTRIBUTOR;
            }
            multiAccCalibrationStatus_t status;
            const bool rawValid = multiAccGetCalibrationStatus(slot, &status) && status.detected && status.readValid;
            float acceleration[3], rawAcceleration[3] = {0};
            memcpy(acceleration, as->value, sizeof(acceleration));
            if (rawValid) memcpy(rawAcceleration, status.uncalibratedG, sizeof(rawAcceleration));
            // Individual accel samples precede board alignment; gyro samples already include it.
            applyBoardAlignment(acceleration); applyBoardAlignment(rawAcceleration);
            bool rawFinite = rawValid;
            for (unsigned axis = 0; axis < 3; axis++) {
                rawFinite &= isfinite(rawAcceleration[axis] * 1000) && fabsf(rawAcceleration[axis] * 1000) <= 2147483520.0f;
                if (!isfinite(gs->value[axis] * 1000) || fabsf(gs->value[axis] * 1000) > 2147483520.0f)
                    imu->flags &= ~(MULTI_IMU_GYRO_VALID | MULTI_IMU_GYRO_CONTRIBUTOR);
                if (!isfinite(acceleration[axis] * 1000) || fabsf(acceleration[axis] * 1000) > 2147483520.0f)
                    imu->flags &= ~(MULTI_IMU_ACC_VALID | MULTI_IMU_ACC_CONTRIBUTOR);
                imu->gyroMdps[axis] = quantize(gs->value[axis] * 1000);
                imu->accMg[axis] = quantize(acceleration[axis] * 1000);
                imu->rawAccMg[axis] = quantize(rawAcceleration[axis] * 1000);
            }
            if (rawFinite) imu->flags |= MULTI_IMU_RAW_ACC_VALID;
        }
    }
    for (unsigned slot = 0; slot < MULTI_BARO_COUNT; slot++) {
        multiBaroTelemetry_t *baro = &state->baro[slot];
        baro->slot = slot; baro->ageUs = UINT32_MAX;
        if (!multiSensorBaroEnabled() || !(baro->hardware = baroGetSensorHardware(slot))) continue;
        const sensorFusionMember_t *member = &multiBaroFusion.member[slot];
        const sensorFusionSample_t *sample = &multiBaroSamples[slot];
        baro->flags = MULTI_BARO_PRESENT;
        baro->ageUs = nowUs - sample->timestampUs;
        baro->trust = trust(member->confidence);
        baro->pressurePa = quantize(sample->value[0]);
        baro->alignedPressurePa = quantize(sample->value[0] + baroGetPressureOffset(slot));
        baro->temperatureCenti = baroGetSensorTemperature(slot);
        const bool pressureValid = member->usable && sample->valid && isfinite(sample->value[0]) &&
            isfinite(member->confidence) && baro->ageUs <= (uint32_t)config->baroMaxAgeMs * 1000;
        if (pressureValid) {
            baro->flags |= MULTI_BARO_VALID;
            if (member->confidence >= SENSOR_FUSION_MIN_CONFIDENCE) baro->flags |= MULTI_BARO_CONTRIBUTOR;
            if (baroGetSensorAltitude(slot, &baro->altitudeCm)) baro->flags |= MULTI_BARO_ALTITUDE_VALID;
        }
    }
}

void multiSensorSerializeTelemetry(sbuf_t *dst, const multiSensorTelemetry_t *state)
{
    sbufWriteU8(dst, MULTI_SENSOR_STATE_VERSION); sbufWriteU8(dst, state->flags);
    sbufWriteU8(dst, MULTI_IMU_COUNT); sbufWriteU8(dst, MULTI_BARO_COUNT);
    sbufWriteU32(dst, state->timestampUs);
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        const multiImuTelemetry_t *imu = &state->imu[i];
        sbufWriteU8(dst, imu->tag); sbufWriteU8(dst, imu->hardware); sbufWriteU8(dst, imu->flags);
        sbufWriteU16(dst, imu->gyroTrust); sbufWriteU16(dst, imu->accTrust);
        sbufWriteU32(dst, imu->gyroAgeUs); sbufWriteU32(dst, imu->accAgeUs);
        for (unsigned axis = 0; axis < 3; axis++) sbufWriteU32(dst, imu->gyroMdps[axis]);
        for (unsigned axis = 0; axis < 3; axis++) sbufWriteU32(dst, imu->accMg[axis]);
        for (unsigned axis = 0; axis < 3; axis++) sbufWriteU32(dst, imu->rawAccMg[axis]);
    }
    for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
        const multiBaroTelemetry_t *baro = &state->baro[i];
        sbufWriteU8(dst, baro->slot); sbufWriteU8(dst, baro->hardware); sbufWriteU8(dst, baro->flags);
        sbufWriteU16(dst, baro->trust); sbufWriteU32(dst, baro->ageUs);
        sbufWriteU32(dst, baro->pressurePa); sbufWriteU32(dst, baro->alignedPressurePa);
        sbufWriteU32(dst, baro->temperatureCenti); sbufWriteU32(dst, baro->altitudeCm);
    }
}
#endif

/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_SENSOR
#include <math.h>
#include <stddef.h>
#include "common/sensor_trust.h"
#include "drivers/time.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/multi_sensor.h"
#include "io/osd_sensor_trust.h"
#include "io/osd_multi_mag.h"

static sensorTrustMonitor_t monitor;
static uint32_t lastUpdateMs;
#ifdef USE_MULTI_MAG
static bool magMessageSelected;
#endif

static bool sampleUsable(const sensorFusionMember_t *member, const sensorFusionSample_t *sample, uint32_t nowUs, uint32_t maxAgeUs)
{
    return member->usable && sample->valid && (uint32_t)(nowUs - sample->timestampUs) <= maxAgeUs;
}

void osdSensorTrustUpdate(void)
{
    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastUpdateMs) < 100) {
        return;
    }
    lastUpdateMs = nowMs;
    const uint32_t nowUs = micros();
#ifdef USE_MULTI_MAG
    osdMultiMagUpdate(nowUs);
#endif
    const multiSensorConfig_t *config = multiSensorConfig();
    sensorTrustInput_t inputs[SENSOR_TRUST_SOURCE_COUNT] = {0};
    if (multiSensorImuEnabled() && gyroIsCalibrationComplete() && accIsCalibrationComplete()) {
        for (unsigned slot = 0; slot < MULTI_IMU_COUNT; slot++) {
            const unsigned tag = gyroGetImuTag(slot);
            const sensorFusionMember_t *gyro = &multiGyroFusion.member[slot];
            if (tag >= MULTI_IMU_COUNT || !gyro->initialized) {
                continue;
            }
            sensorTrustInput_t *input = &inputs[tag];
            input->active = true;
            input->confidence = gyro->confidence;
            input->valid = isfinite(gyro->confidence) &&
                sampleUsable(gyro, &multiGyroSamples[slot], nowUs, (uint32_t)config->imuMaxAgeMs * 1000);
            if (config->accCalibratedMask & (1u << tag)) {
                const sensorFusionMember_t *acc = &multiAccFusion.member[slot];
                input->valid &= isfinite(acc->confidence) &&
                    sampleUsable(acc, &multiAccSamples[slot], nowUs, (uint32_t)config->imuMaxAgeMs * 1000);
                input->confidence = fminf(input->confidence, acc->confidence);
            }
        }
    }
    if (multiSensorBaroEnabled()) {
        for (unsigned i = 0; i < MULTI_BARO_COUNT; i++) {
            const sensorFusionMember_t *baro = &multiBaroFusion.member[i];
            inputs[MULTI_IMU_COUNT + i] = (sensorTrustInput_t) {
                .active = baro->initialized,
                .valid = sampleUsable(baro, &multiBaroSamples[i], nowUs, (uint32_t)config->baroMaxAgeMs * 1000),
                .confidence = baro->confidence,
            };
        }
    }
    sensorTrustUpdate(&monitor, inputs, nowMs);
}

const char *osdSensorTrustMessage(void)
{
    static const char *const lost[SENSOR_TRUST_SOURCE_COUNT] = {
        "IMU 1 TRUST LOST", "IMU 2 TRUST LOST", "IMU 3 TRUST LOST", "MS5611 TRUST LOST", "SPL06 TRUST LOST",
    };
    static const char *const restored[SENSOR_TRUST_SOURCE_COUNT] = {
        "IMU 1 TRUST RESTORED", "IMU 2 TRUST RESTORED", "IMU 3 TRUST RESTORED", "MS5611 TRUST RESTORED", "SPL06 TRUST RESTORED",
    };
    const sensorTrustEvent_t *event = sensorTrustPeek(&monitor, millis());
    if (event) {
#ifdef USE_MULTI_MAG
        magMessageSelected = false;
        osdMultiMagSetVisible(false);
#endif
        return event->restored ? restored[event->source] : lost[event->source];
    }
#ifdef USE_MULTI_MAG
    const char *message = osdMultiMagMessage(millis());
    magMessageSelected = message != NULL;
    return message;
#else
    return NULL;
#endif
}

void osdSensorTrustSetVisible(bool visible)
{
    sensorTrustSetVisible(&monitor, visible);
#ifdef USE_MULTI_MAG
    osdMultiMagSetVisible(visible && magMessageSelected);
#endif
}
#endif

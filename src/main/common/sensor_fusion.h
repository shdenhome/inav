/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_FUSION_MAX_SENSORS 3
#define SENSOR_FUSION_AXES 3
// Below this confidence the source is quarantined until gradual recovery.
#define SENSOR_FUSION_MIN_CONFIDENCE 0.1f

typedef struct sensorFusionSample_s {
    float value[SENSOR_FUSION_AXES];
    uint32_t timestampUs;
    bool valid;
} sensorFusionSample_t;

typedef struct sensorFusionMember_s {
    float confidence;
    float accepted[SENSOR_FUSION_AXES];
    float velocity[SENSOR_FUSION_AXES];
    uint32_t acceptedAtUs;
    uint32_t processedAtUs;
    bool initialized;
    bool usable;
    bool historyExpired;
} sensorFusionMember_t;

typedef struct sensorFusion_s {
    sensorFusionMember_t member[SENSOR_FUSION_MAX_SENSORS];
    float output[SENSOR_FUSION_AXES];
    bool healthy;
    uint8_t contributors;
} sensorFusion_t;

typedef struct sensorFusionConfig_s {
    float deviation;            // Common physical units, not ADC counts.
    float recoveryTimeSeconds;
    uint32_t maxAgeUs;
} sensorFusionConfig_t;

void sensorFusionReset(sensorFusion_t *fusion);
// Invalid/future/stale/nonfinite samples never contribute. Each sensor has one vote,
// regardless of how frequently the consumer calls this function. Failure leaves the
// last output intact, sets healthy=false and must be propagated by the caller.
bool sensorFusionUpdate(sensorFusion_t *fusion, const sensorFusionSample_t *samples,
    unsigned count, unsigned axes, uint32_t nowUs, const sensorFusionConfig_t *config);

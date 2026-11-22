/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define MULTI_MAG_MAX_SOURCES 12

typedef struct magFusionSample_s {
    uint32_t id, timestampUs;
    float field[3], earthField[3];
    bool valid, earthValid;
} magFusionSample_t;
typedef struct magFusionMember_s {
    float confidence;
    float earthReference[3];
    uint32_t processedAtUs, badSinceUs, goodSinceUs;
    bool usable, contributing, initialized, quarantined;
    bool badPending, goodPending, earthReferenceValid;
} magFusionMember_t;
typedef struct magFusion_s {
    magFusionMember_t member[MULTI_MAG_MAX_SOURCES];
    float output[3];
    uint32_t referenceId, fallbackId;
    uint8_t contributors;
    bool healthy;
} magFusion_t;
typedef struct magFusionConfig_s {
    uint32_t maxAgeUs, rejectUs, goodUs;
    float agreementDegrees, rejectDegrees, recoverySeconds;
    float fieldMin, fieldMax;
} magFusionConfig_t;
void magFusionReset(magFusion_t *fusion);
bool magFusionUpdate(magFusion_t *fusion, const magFusionSample_t samples[MULTI_MAG_MAX_SOURCES],
    uint32_t nowUs, const magFusionConfig_t *config);

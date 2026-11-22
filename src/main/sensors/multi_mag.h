/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "common/time.h"
#include "common/mag_fusion.h"
#include "common/streambuf.h"
#include "config/parameter_group.h"

#ifdef USE_MULTI_MAG
#define MULTI_MAG_PAGE_SIZE 4
#define MULTI_MAG_STATE_VERSION 1
#define MULTI_MAG_CAL_VALID 1
#define MULTI_MAG_CAL_TAILSITTER 2

typedef enum { MAG_CAL_NEEDED, MAG_CAL_RUNNING, MAG_CAL_OK, MAG_CAL_FAILED, MAG_CAL_MISSING } multiMagCalibrationStatus_e;
typedef enum { MAG_CAL_ERROR_NONE, MAG_CAL_ERROR_READ, MAG_CAL_ERROR_COVERAGE, MAG_CAL_ERROR_SOLVER, MAG_CAL_ERROR_GAIN, MAG_CAL_ERROR_ORIENTATION } multiMagCalibrationError_e;
typedef struct multiMagCalibration_s {
    uint32_t id;
    int16_t zero[3], gain[3], angles[3];
    uint8_t flags, orientationQuality;
} multiMagCalibration_t;
typedef struct multiMagConfig_s {
    uint8_t enabled;
    uint16_t maxAgeMs, rejectMs, goodMs, recoveryMs;
    uint8_t agreementDegrees, rejectDegrees;
    uint16_t fieldMinPercent, fieldMaxPercent;
    multiMagCalibration_t calibration[MULTI_MAG_MAX_SOURCES];
} multiMagConfig_t;
PG_DECLARE(multiMagConfig_t, multiMagConfig);

enum {
    MULTI_MAG_DETECTED=1, MULTI_MAG_READABLE=2, MULTI_MAG_FRESH=4,
    MULTI_MAG_CALIBRATED=8, MULTI_MAG_CALIBRATING=16, MULTI_MAG_CONTRIBUTING=32,
    MULTI_MAG_FALLBACK=64, MULTI_MAG_POSITIVE_TRUST=128, MULTI_MAG_ORIENTATION_VALID=256,
    MULTI_MAG_CALIBRATION_FAILED=512, MULTI_MAG_READ_ERROR=1024,
};
typedef struct multiMagSourceState_s {
    uint32_t id, ageUs;
    uint8_t slot, hardware, bus, address;
    uint16_t flags, confidence;
    int16_t raw[3], angles[3];
    float aligned[3];
    uint8_t calibrationStatus, orientationQuality, failureCode;
    uint16_t sampleCount;
} multiMagSourceState_t;

bool multiMagEnabled(void);
bool multiMagInit(void);
void multiMagUpdate(timeUs_t nowUs);
bool multiMagIsHealthy(void);
bool multiMagIsReady(void);
bool multiMagIsCalibrated(void);
bool multiMagIsCalibrating(void);
bool multiMagDetected(unsigned slot);
bool multiMagCaptureSource(unsigned slot, uint32_t nowUs, multiMagSourceState_t *state);
uint32_t multiMagReferenceId(void);
uint32_t multiMagFallbackId(void);
uint32_t multiMagCalibrationRunId(void);
unsigned multiMagCount(void);
void multiMagSerializeState(sbuf_t *dst, uint8_t cursor, uint16_t expectedGeneration);
#else
static inline bool multiMagEnabled(void) { return false; }
#endif

/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config/parameter_group.h"
#include "common/sensor_fusion.h"

#ifdef USE_MULTI_SENSOR
#define MULTI_IMU_COUNT 3
#define MULTI_BARO_COUNT 2

typedef struct multiSensorConfig_s {
    uint8_t imuEnabled;
    uint8_t baroEnabled;
    uint16_t gyroDeviationDps;
    uint16_t accDeviationMg;
    uint16_t baroDeviationPa;
    uint16_t recoveryMs;
    uint16_t imuMaxAgeMs;
    uint16_t baroMaxAgeMs;
    uint8_t accCalibratedMask;
    int16_t accZero[MULTI_IMU_COUNT][3]; // Body axes in units of 1/4096 g.
    int16_t accGain[MULTI_IMU_COUNT][3]; // 4096 = unity.
} multiSensorConfig_t;
PG_DECLARE(multiSensorConfig_t, multiSensorConfig);

void multiSensorLatchImuMode(void);
void multiSensorLatchBaroMode(void);
bool multiSensorImuEnabled(void);
bool multiSensorBaroEnabled(void);

extern sensorFusion_t multiGyroFusion;
extern sensorFusion_t multiAccFusion;
extern sensorFusion_t multiBaroFusion;
extern sensorFusionSample_t multiGyroSamples[MULTI_IMU_COUNT];
extern sensorFusionSample_t multiAccSamples[MULTI_IMU_COUNT];
extern sensorFusionSample_t multiBaroSamples[MULTI_BARO_COUNT];
sensorFusionConfig_t multiSensorFusionConfig(float deviation, bool barometer);
#endif

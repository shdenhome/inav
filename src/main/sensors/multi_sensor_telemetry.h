/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "common/streambuf.h"
#include "sensors/multi_sensor.h"
#ifdef USE_MULTI_SENSOR
#define MULTI_SENSOR_STATE_VERSION 1
#define MULTI_SENSOR_STATE_BYTES 211
enum {
    MULTI_IMU_PRESENT = 1,
    MULTI_IMU_GYRO_VALID = 2,
    MULTI_IMU_ACC_VALID = 4,
    MULTI_IMU_ACC_CALIBRATED = 8,
    MULTI_IMU_RAW_ACC_VALID = 16,
    MULTI_IMU_GYRO_CONTRIBUTOR = 32,
    MULTI_IMU_ACC_CONTRIBUTOR = 64,
};
enum {
    MULTI_BARO_PRESENT = 1,
    MULTI_BARO_VALID = 2,
    MULTI_BARO_ALTITUDE_VALID = 4,
    MULTI_BARO_CONTRIBUTOR = 8,
};
typedef struct multiImuTelemetry_s {
    uint8_t tag, hardware, flags;
    uint16_t gyroTrust, accTrust; // 0..1000
    uint32_t gyroAgeUs, accAgeUs;
    int32_t gyroMdps[3], accMg[3], rawAccMg[3];
} multiImuTelemetry_t;
typedef struct multiBaroTelemetry_s {
    uint8_t slot, hardware, flags;
    uint16_t trust;
    uint32_t ageUs;
    int32_t pressurePa, alignedPressurePa, temperatureCenti, altitudeCm;
} multiBaroTelemetry_t;
typedef struct multiSensorTelemetry_s {
    uint32_t timestampUs;
    uint8_t flags; // bit0: multi IMU, bit1: multi barometer
    multiImuTelemetry_t imu[MULTI_IMU_COUNT]; // physical tag order
    multiBaroTelemetry_t baro[MULTI_BARO_COUNT];
} multiSensorTelemetry_t;
void multiSensorCaptureTelemetry(multiSensorTelemetry_t *state, uint32_t nowUs);
void multiSensorSerializeTelemetry(sbuf_t *dst, const multiSensorTelemetry_t *state);
#endif

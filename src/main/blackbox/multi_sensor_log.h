/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
#ifdef USE_MULTI_SENSOR
enum { MULTI_LOG_IMU = 1, MULTI_LOG_GYRO = 2, MULTI_LOG_ACC = 4, MULTI_LOG_BARO = 8 };
typedef struct multiSensorLogState_s {
    int32_t imuType[3];
    int32_t imuFlags[3];
    int32_t imuGyroTrust[3];
    int32_t imuGyroAgeMs[3];
    int32_t imuGyroMdps[9];
    int32_t imuAccTrust[3];
    int32_t imuAccAgeMs[3];
    int32_t imuAccMg[9];
    int32_t imuRawAccMg[9];
    int32_t baroType[2];
    int32_t baroFlags[2];
    int32_t baroTrust[2];
    int32_t baroAgeMs[2];
    int32_t baroPressurePa[2];
    int32_t baroAlignedPa[2];
    int32_t baroTempCenti[2];
    int32_t baroAltitudeCm[2];
} multiSensorLogState_t;

// Independent field lists keep main frames compact and diagnostics in standard S frames.
#define MULTI_SENSOR_MAIN_LOG_FIELDS(X) \
    X(imuGyroMdps, 0, imuGyroMdps[0], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 1, imuGyroMdps[1], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 2, imuGyroMdps[2], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 3, imuGyroMdps[3], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 4, imuGyroMdps[4], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 5, imuGyroMdps[5], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 6, imuGyroMdps[6], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 7, imuGyroMdps[7], MULTI_LOG_GYRO) \
    X(imuGyroMdps, 8, imuGyroMdps[8], MULTI_LOG_GYRO) \
    X(imuAccMg, 0, imuAccMg[0], MULTI_LOG_ACC) \
    X(imuAccMg, 1, imuAccMg[1], MULTI_LOG_ACC) \
    X(imuAccMg, 2, imuAccMg[2], MULTI_LOG_ACC) \
    X(imuAccMg, 3, imuAccMg[3], MULTI_LOG_ACC) \
    X(imuAccMg, 4, imuAccMg[4], MULTI_LOG_ACC) \
    X(imuAccMg, 5, imuAccMg[5], MULTI_LOG_ACC) \
    X(imuAccMg, 6, imuAccMg[6], MULTI_LOG_ACC) \
    X(imuAccMg, 7, imuAccMg[7], MULTI_LOG_ACC) \
    X(imuAccMg, 8, imuAccMg[8], MULTI_LOG_ACC) \
    X(baroPressurePa, 0, baroPressurePa[0], MULTI_LOG_BARO) \
    X(baroPressurePa, 1, baroPressurePa[1], MULTI_LOG_BARO)

#define MULTI_SENSOR_SLOW_LOG_FIELDS(X) \
    X(imuType, 0, imuType[0], MULTI_LOG_IMU) \
    X(imuType, 1, imuType[1], MULTI_LOG_IMU) \
    X(imuType, 2, imuType[2], MULTI_LOG_IMU) \
    X(imuFlags, 0, imuFlags[0], MULTI_LOG_IMU) \
    X(imuFlags, 1, imuFlags[1], MULTI_LOG_IMU) \
    X(imuFlags, 2, imuFlags[2], MULTI_LOG_IMU) \
    X(imuGyroTrust, 0, imuGyroTrust[0], MULTI_LOG_IMU) \
    X(imuGyroTrust, 1, imuGyroTrust[1], MULTI_LOG_IMU) \
    X(imuGyroTrust, 2, imuGyroTrust[2], MULTI_LOG_IMU) \
    X(imuGyroAgeMs, 0, imuGyroAgeMs[0], MULTI_LOG_IMU) \
    X(imuGyroAgeMs, 1, imuGyroAgeMs[1], MULTI_LOG_IMU) \
    X(imuGyroAgeMs, 2, imuGyroAgeMs[2], MULTI_LOG_IMU) \
    X(imuAccTrust, 0, imuAccTrust[0], MULTI_LOG_ACC) \
    X(imuAccTrust, 1, imuAccTrust[1], MULTI_LOG_ACC) \
    X(imuAccTrust, 2, imuAccTrust[2], MULTI_LOG_ACC) \
    X(imuAccAgeMs, 0, imuAccAgeMs[0], MULTI_LOG_ACC) \
    X(imuAccAgeMs, 1, imuAccAgeMs[1], MULTI_LOG_ACC) \
    X(imuAccAgeMs, 2, imuAccAgeMs[2], MULTI_LOG_ACC) \
    X(imuRawAccMg, 0, imuRawAccMg[0], MULTI_LOG_ACC) \
    X(imuRawAccMg, 1, imuRawAccMg[1], MULTI_LOG_ACC) \
    X(imuRawAccMg, 2, imuRawAccMg[2], MULTI_LOG_ACC) \
    X(imuRawAccMg, 3, imuRawAccMg[3], MULTI_LOG_ACC) \
    X(imuRawAccMg, 4, imuRawAccMg[4], MULTI_LOG_ACC) \
    X(imuRawAccMg, 5, imuRawAccMg[5], MULTI_LOG_ACC) \
    X(imuRawAccMg, 6, imuRawAccMg[6], MULTI_LOG_ACC) \
    X(imuRawAccMg, 7, imuRawAccMg[7], MULTI_LOG_ACC) \
    X(imuRawAccMg, 8, imuRawAccMg[8], MULTI_LOG_ACC) \
    X(baroType, 0, baroType[0], MULTI_LOG_BARO) \
    X(baroType, 1, baroType[1], MULTI_LOG_BARO) \
    X(baroFlags, 0, baroFlags[0], MULTI_LOG_BARO) \
    X(baroFlags, 1, baroFlags[1], MULTI_LOG_BARO) \
    X(baroTrust, 0, baroTrust[0], MULTI_LOG_BARO) \
    X(baroTrust, 1, baroTrust[1], MULTI_LOG_BARO) \
    X(baroAgeMs, 0, baroAgeMs[0], MULTI_LOG_BARO) \
    X(baroAgeMs, 1, baroAgeMs[1], MULTI_LOG_BARO) \
    X(baroAlignedPa, 0, baroAlignedPa[0], MULTI_LOG_BARO) \
    X(baroAlignedPa, 1, baroAlignedPa[1], MULTI_LOG_BARO) \
    X(baroTempCenti, 0, baroTempCenti[0], MULTI_LOG_BARO) \
    X(baroTempCenti, 1, baroTempCenti[1], MULTI_LOG_BARO) \
    X(baroAltitudeCm, 0, baroAltitudeCm[0], MULTI_LOG_BARO) \
    X(baroAltitudeCm, 1, baroAltitudeCm[1], MULTI_LOG_BARO)

void multiSensorLogCapture(multiSensorLogState_t *state, uint32_t nowUs);
void multiSensorLogWriteMain(const multiSensorLogState_t *state, const multiSensorLogState_t *previous, unsigned groups);
void multiSensorLogWriteSlow(const multiSensorLogState_t *state, unsigned groups);
#endif

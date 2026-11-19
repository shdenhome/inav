/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_SENSOR
#include <math.h>
#include <string.h>
#include "common/calibration.h"
#include "common/utils.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_icm42605.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/multi_sensor.h"
#include "sensors/multi_acc.h"

static accDev_t devices[MULTI_IMU_COUNT];
static bool detected[MULTI_IMU_COUNT];
static uint8_t hardware[MULTI_IMU_COUNT];
static bool gyroCached[MULTI_IMU_COUNT];
static float raw[MULTI_IMU_COUNT][3];
static uint32_t rawAt[MULTI_IMU_COUNT];
static bool rawValid[MULTI_IMU_COUNT];
static zeroCalibrationVector_t calibration[MULTI_IMU_COUNT];
static float faces[MULTI_IMU_COUNT][6][3];
static bool faceValid[MULTI_IMU_COUNT][6];
static bool collecting;
static int calibrationFace;
static unsigned activeTags;

bool multiAccInit(void)
{
    memset(devices, 0, sizeof(devices));
    memset(detected, 0, sizeof(detected));
    memset(hardware, 0, sizeof(hardware));
    memset(gyroCached, 0, sizeof(gyroCached));
    memset(rawValid, 0, sizeof(rawValid));
    memset(faceValid, 0, sizeof(faceValid));
    memset(multiAccSamples, 0, sizeof(multiAccSamples));
    sensorFusionReset(&multiAccFusion);
    collecting = false;
    activeTags = 0;
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        const uint8_t tag = gyroGetImuTag(i);
        if (tag >= MULTI_IMU_COUNT) {
            continue;
        }
        devices[i].imuSensorToUse = tag;
        // Open the already detected gyro's bus context. Do not probe/reset its chip.
        detected[i] = icm42605AccDetect(&devices[i]);
        hardware[i] = detected[i] ? ACC_ICM42605 : ACC_NONE;
        if (!detected[i]) {
            detected[i] = mpu6500AccDetect(&devices[i]);
            hardware[i] = detected[i] ? ACC_MPU6500 : ACC_NONE;
            gyroCached[i] = detected[i];
        }
        if (detected[i]) {
            devices[i].initFn(&devices[i]);
            activeTags |= 1u << tag;
        }
    }
    return activeTags != 0;
}

uint8_t multiAccGetHardware(unsigned tag)
{
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (detected[i] && devices[i].imuSensorToUse == tag) {
            return hardware[i];
        }
    }
    return ACC_NONE;
}

bool multiAccCalibrationValid(void)
{
    return activeTags && (multiSensorConfig()->accCalibratedMask & activeTags) == activeTags;
}

bool multiAccCalibrationComplete(void)
{
    return !collecting;
}

unsigned multiAccCalibrationFaces(void)
{
    unsigned mask = 0x3f;
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (detected[i]) {
            for (unsigned face = 0; face < 6; face++) {
                if (!faceValid[i][face]) {
                    mask &= ~(1u << face);
                }
            }
        }
    }
    return multiAccCalibrationValid() ? 0x3f : (activeTags ? mask : 0);
}

static int orientation(const float vector[3])
{
    if (fabsf(vector[2]) / 1.5f > fabsf(vector[0]) && fabsf(vector[2]) / 1.5f > fabsf(vector[1])) {
        return vector[2] > 0 ? 0 : 1;
    }
    if (fabsf(vector[0]) / 1.5f > fabsf(vector[1]) && fabsf(vector[0]) / 1.5f > fabsf(vector[2])) {
        return vector[0] > 0 ? 2 : 3;
    }
    if (fabsf(vector[1]) / 1.5f > fabsf(vector[0]) && fabsf(vector[1]) / 1.5f > fabsf(vector[2])) {
        return vector[1] > 0 ? 4 : 5;
    }
    return -1;
}

bool multiAccGetCalibrationStatus(unsigned slot, multiAccCalibrationStatus_t *status)
{
    if (slot >= MULTI_IMU_COUNT || !status) {
        return false;
    }
    memset(status, 0, sizeof(*status));
    status->detected = detected[slot];
    status->readValid = detected[slot] && rawValid[slot] &&
        (uint32_t)(micros() - rawAt[slot]) <= (uint32_t)multiSensorConfig()->imuMaxAgeMs * 1000;
    status->collecting = collecting;
    status->orientation = status->readValid ? orientation(raw[slot]) : -1;
    status->collectingFace = collecting ? calibrationFace : -1;
    for (unsigned face = 0; face < 6; face++) {
        if (detected[slot] && faceValid[slot][face]) {
            status->completedFaces |= 1u << face;
        }
    }
    for (unsigned axis = 0; axis < 3; axis++) {
        status->uncalibratedG[axis] = raw[slot][axis] / 4096.0f;
    }
    return true;
}

void multiAccStartCalibration(void)
{
    if (collecting) {
        return;
    }
    int face = -1;
    const uint32_t now = micros();
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (!detected[i]) {
            continue;
        }
        if (!rawValid[i] || (uint32_t)(now - rawAt[i]) > (uint32_t)multiSensorConfig()->imuMaxAgeMs * 1000) {
            return;
        }
        const int next = orientation(raw[i]);
        if (next < 0 || (face >= 0 && face != next)) {
            return;
        }
        face = next;
    }
    if (face < 0 || (multiAccCalibrationValid() && face != 0)) {
        return;
    }
    if (face == 0) {
        memset(faceValid, 0, sizeof(faceValid));
        multiSensorConfigMutable()->accCalibratedMask = 0;
        sensorFusionReset(&multiAccFusion);
        DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
    calibrationFace = face;
    collecting = true;
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (detected[i]) {
            faceValid[i][face] = false;
            zeroCalibrationStartV(&calibration[i], CALIBRATING_ACC_TIME_MS, 4096.0f * 0.05f, true);
        }
    }
}

static void finishCalibration(void)
{
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (!detected[i]) {
            continue;
        }
        for (unsigned face = 0; face < 6; face++) {
            if (!faceValid[i][face]) {
                return;
            }
        }
    }
    int16_t offsets[MULTI_IMU_COUNT][3] = {{0}};
    int16_t gains[MULTI_IMU_COUNT][3] = {{0}};
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (!detected[i]) {
            continue;
        }
        sensorCalibrationState_t state;
        float zero[3], gain[3];
        sensorCalibrationResetState(&state);
        for (unsigned face = 0; face < 6; face++) {
            sensorCalibrationPushSampleForOffsetCalculation(&state, faces[i][face]);
        }
        if (!sensorCalibrationSolveForOffset(&state, zero)) {
            memset(faceValid, 0, sizeof(faceValid));
            return;
        }
        sensorCalibrationResetState(&state);
        for (unsigned face = 0; face < 6; face++) {
            float sample[3];
            for (unsigned axis = 0; axis < 3; axis++) {
                sample[axis] = faces[i][face][axis] - zero[axis];
            }
            // Face order is +/-Z, +/-X, +/-Y in body coordinates.
            const unsigned axis = (face / 2 + 2) % 3;
            sensorCalibrationPushSampleForScaleCalculation(&state, axis, sample, 4096);
        }
        if (!sensorCalibrationSolveForScale(&state, gain)) {
            memset(faceValid, 0, sizeof(faceValid));
            return;
        }
        for (unsigned axis = 0; axis < 3; axis++) {
            if (!isfinite(zero[axis]) || fabsf(zero[axis]) > 2048 || !isfinite(gain[axis]) || gain[axis] < 0.5f || gain[axis] > 2) {
                memset(faceValid, 0, sizeof(faceValid));
                return;
            }
            offsets[i][axis] = lrintf(zero[axis]);
            gains[i][axis] = lrintf(gain[axis] * 4096);
        }
    }
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (detected[i]) {
            const uint8_t tag = devices[i].imuSensorToUse;
            memcpy(multiSensorConfigMutable()->accZero[tag], offsets[i], sizeof(offsets[i]));
            memcpy(multiSensorConfigMutable()->accGain[tag], gains[i], sizeof(gains[i]));
        }
    }
    multiSensorConfigMutable()->accCalibratedMask = activeTags;
    saveConfigAndNotify();
    sensorFusionReset(&multiAccFusion);
}

bool multiAccUpdate(float output[3])
{
    const uint32_t now = micros();
    bool calibrationFinished = collecting;
    for (unsigned i = 0; i < MULTI_IMU_COUNT; i++) {
        if (!detected[i]) {
            continue;
        }
        const uint32_t sampleTime = gyroCached[i] ? gyroGetImuSampleTime(i) : now;
        const bool newSample = !rawValid[i] || sampleTime != rawAt[i];
        rawValid[i] = (uint32_t)(now - sampleTime) <= (uint32_t)multiSensorConfig()->imuMaxAgeMs * 1000 &&
            devices[i].readFn(&devices[i]);
        if (!rawValid[i]) {
            multiAccSamples[i].valid = false;
            calibrationFinished = false;
            if (collecting) {
                zeroCalibrationStartV(&calibration[i], CALIBRATING_ACC_TIME_MS, 4096.0f * 0.05f, true);
            }
            continue;
        }
        float aligned[3];
        applySensorAlignment(aligned, devices[i].ADCRaw, devices[i].accAlign);
        for (unsigned axis = 0; axis < 3; axis++) {
            raw[i][axis] = aligned[axis] * (4096.0f / devices[i].acc_1G);
        }
        rawAt[i] = sampleTime;
        if (collecting) {
            if (newSample && !zeroCalibrationIsCompleteV(&calibration[i])) {
                fpVector3_t vector;
                memcpy(vector.v, raw[i], sizeof(vector.v));
                zeroCalibrationAddValueV(&calibration[i], &vector);
            }
            if (zeroCalibrationIsCompleteV(&calibration[i])) {
                if (zeroCalibrationIsSuccessfulV(&calibration[i]) && orientation(raw[i]) == calibrationFace) {
                    fpVector3_t vector;
                    zeroCalibrationGetZeroV(&calibration[i], &vector);
                    memcpy(faces[i][calibrationFace], vector.v, sizeof(vector.v));
                    faceValid[i][calibrationFace] = true;
                }
            } else {
                calibrationFinished = false;
            }
        }
        const uint8_t tag = devices[i].imuSensorToUse;
        multiAccSamples[i].valid = (multiSensorConfig()->accCalibratedMask & (1u << tag)) && !collecting;
        for (unsigned axis = 0; axis < 3; axis++) {
            multiAccSamples[i].value[axis] = (raw[i][axis] - multiSensorConfig()->accZero[tag][axis]) *
                (multiSensorConfig()->accGain[tag][axis] / (4096.0f * 4096.0f));
        }
        multiAccSamples[i].timestampUs = sampleTime;
    }
    if (collecting && calibrationFinished) {
        collecting = false;
        beeperConfirmationBeeps(2);
        finishCalibration();
    }
    const sensorFusionConfig_t config = multiSensorFusionConfig(multiSensorConfig()->accDeviationMg * 0.001f, false);
    if (!sensorFusionUpdate(&multiAccFusion, multiAccSamples, MULTI_IMU_COUNT, 3, now, &config)) {
        return false;
    }
    memcpy(output, multiAccFusion.output, sizeof(multiAccFusion.output));
    return true;
}
#endif

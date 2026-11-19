/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef USE_MULTI_SENSOR
typedef struct multiAccCalibrationStatus_s {
    bool detected;
    bool readValid;
    bool collecting;
    int8_t orientation;
    int8_t collectingFace;
    uint8_t completedFaces;
    float uncalibratedG[3];
} multiAccCalibrationStatus_t;

bool multiAccGetCalibrationStatus(unsigned slot, multiAccCalibrationStatus_t *status);
uint8_t multiAccGetHardware(unsigned tag);
bool multiAccInit(void);
bool multiAccUpdate(float output[3]);
void multiAccStartCalibration(void);
bool multiAccCalibrationComplete(void);
bool multiAccCalibrationValid(void);
unsigned multiAccCalibrationFaces(void);
#endif

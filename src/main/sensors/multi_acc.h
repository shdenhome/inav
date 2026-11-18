/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#ifdef USE_MULTI_SENSOR
bool multiAccInit(void);
bool multiAccUpdate(float output[3]);
void multiAccStartCalibration(void);
bool multiAccCalibrationComplete(void);
bool multiAccCalibrationValid(void);
unsigned multiAccCalibrationFaces(void);
#endif

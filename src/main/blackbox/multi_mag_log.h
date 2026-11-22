/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdint.h>
#include "sensors/multi_mag.h"
#ifdef USE_MULTI_MAG
#define MULTI_MAG_LOG_FIELDS(X) X(0) X(1) X(2) X(3) X(4) X(5) X(6) X(7) X(8) X(9) X(10) X(11)
typedef struct multiMagLogState_s { int16_t value[MULTI_MAG_MAX_SOURCES][4]; } multiMagLogState_t;
void multiMagLogCapture(multiMagLogState_t *state, uint32_t nowUs);
void multiMagLogWrite(const multiMagLogState_t *state, uint16_t mask);
#endif

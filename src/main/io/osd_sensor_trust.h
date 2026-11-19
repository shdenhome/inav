/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>

#ifdef USE_MULTI_SENSOR
void osdSensorTrustUpdate(void);
const char *osdSensorTrustMessage(void);
void osdSensorTrustSetVisible(bool visible);
#endif

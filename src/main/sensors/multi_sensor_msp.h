/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "common/streambuf.h"
#ifdef USE_MULTI_SENSOR
void multiSensorSerializeInfo(sbuf_t *dst);
#endif

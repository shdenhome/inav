/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef USE_MULTI_MAG
void osdMultiMagUpdate(uint32_t nowUs);
const char *osdMultiMagMessage(uint32_t nowMs);
void osdMultiMagSetVisible(bool visible);
#endif

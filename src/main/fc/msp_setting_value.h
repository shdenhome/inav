/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "common/streambuf.h"

// Validate before updating a setting. Widening preserves signed minima and
// unsigned maxima without signed/unsigned comparison surprises.
static inline bool mspReadSettingU16(sbuf_t *src, int32_t minimum, uint32_t maximum, uint16_t *destination)
{
    uint16_t value;
    if (!sbufReadU16Safe(&value, src) || (int32_t)value < minimum || (uint32_t)value > maximum) {
        return false;
    }
    *destination = value;
    return true;
}

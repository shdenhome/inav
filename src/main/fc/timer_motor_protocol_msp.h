/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "common/streambuf.h"

// Decode into scratch storage first: a malformed tail must never partially apply.
// Resource/compile-time capability validation is performed by the caller.
static inline bool timerMotorProtocolsDecode(sbuf_t *src, uint8_t count,
    const uint8_t *current, uint8_t *proposed)
{
    if (sbufBytesRemaining(src) < 2 || sbufReadU8(src) != 1) {
        return false;
    }
    const uint8_t pairs = sbufReadU8(src);
    if (pairs > count || sbufBytesRemaining(src) != pairs * 2) {
        return false;
    }
    uint8_t candidate[256];
    uint8_t seen[256] = {0};
    memcpy(candidate, current, count);
    for (unsigned i = 0; i < pairs; i++) {
        const uint8_t timer = sbufReadU8(src);
        const uint8_t protocol = sbufReadU8(src);
        if (timer >= count || seen[timer] ||
            !(protocol == 255 || protocol == 0 || (protocol >= 4 && protocol <= 6))) {
            return false;
        }
        seen[timer] = 1;
        candidate[timer] = protocol;
    }
    memcpy(proposed, candidate, count);
    return true;
}

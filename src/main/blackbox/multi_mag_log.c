/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#if defined(USE_MULTI_MAG) && defined(USE_BLACKBOX)
#include <math.h>
#include <string.h>
#include "blackbox/multi_mag_log.h"
#include "blackbox/blackbox_encoding.h"
void multiMagLogCapture(multiMagLogState_t *state, uint32_t nowUs)
{
    memset(state, 0, sizeof(*state));
    for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) {
        multiMagSourceState_t source;
        if (!multiMagCaptureSource(i, nowUs, &source)) continue;
        unsigned flags = 0;
        if ((source.flags & (MULTI_MAG_READABLE | MULTI_MAG_FRESH)) == (MULTI_MAG_READABLE | MULTI_MAG_FRESH)) flags |= 1;
        if (source.flags & MULTI_MAG_CONTRIBUTING) flags |= 2;
        if (source.flags & MULTI_MAG_FALLBACK) flags |= 4;
        if (source.flags & MULTI_MAG_CALIBRATED) flags |= 8;
        if (source.flags & MULTI_MAG_CALIBRATING) flags |= 16;
        for (unsigned axis = 0; axis < 3; axis++) {
            float value = source.aligned[axis] / 16;
            if (!isfinite(value)) { value = 0; flags |= 32; }
            if (value > 127) { value = 127; flags |= 32; }
            if (value < -127) { value = -127; flags |= 32; }
            state->value[i][axis] = lrintf(value);
        }
        const unsigned confidence = source.confidence > 1000 ? 1000 : source.confidence;
        state->value[i][3] = ((confidence * 255 + 999) / 1000) | (flags << 8);
    }
}
void multiMagLogWrite(const multiMagLogState_t *state, uint16_t mask)
{
    for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) if (mask & (1u << i)) {
        int32_t values[4];
        for (unsigned j = 0; j < 4; j++) values[j] = state->value[i][j];
        blackboxWriteTag8_4S16(values);
    }
}
#endif

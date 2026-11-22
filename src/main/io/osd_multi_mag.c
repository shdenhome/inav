/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_MAG
#include <string.h>
#include "io/osd_multi_mag.h"
#include "sensors/multi_mag.h"
typedef struct { uint32_t id; bool known, lost; } magTrustState_t;
typedef struct { uint8_t slot; bool restored; } magTrustEvent_t;
static magTrustState_t states[MULTI_MAG_MAX_SOURCES];
static magTrustEvent_t events[2 * MULTI_MAG_MAX_SOURCES];
static unsigned count;
static uint32_t visibleMs, lastDrawMs;
static bool wasVisible;
static unsigned fallbackSlot;
static bool fallback;
static char message[32];
static void popEvent(void)
{
    if (count) { count--; memmove(events, events + 1, count * sizeof(events[0])); }
    visibleMs = 0; wasVisible = false;
}
static void pushEvent(unsigned slot, bool restored)
{
    if (count == 2 * MULTI_MAG_MAX_SOURCES) popEvent();
    events[count++] = (magTrustEvent_t){ slot, restored };
}
void osdMultiMagUpdate(uint32_t nowUs)
{
    fallback = false;
    if (!multiMagEnabled()) { memset(states, 0, sizeof(states)); count = visibleMs = 0; wasVisible = false; return; }
    for (unsigned i = 0; i < MULTI_MAG_MAX_SOURCES; i++) {
        multiMagSourceState_t s;
        if (!multiMagCaptureSource(i, nowUs, &s)) continue;
        if (s.flags & MULTI_MAG_FALLBACK) { fallback = true; fallbackSlot = i; }
        magTrustState_t *state = &states[i];
        // Calibration is intentional unavailability, not trust loss.
        if ((s.flags & MULTI_MAG_CALIBRATING) || !(s.flags & MULTI_MAG_CALIBRATED)) continue;
        if (state->id != s.id) *state = (magTrustState_t){ .id = s.id };
        const bool lost = s.confidence == 0;
        if (!state->known) {
            state->known = true; state->lost = lost;
            if (lost) pushEvent(i, false);
        } else if (lost != state->lost) {
            state->lost = lost; pushEvent(i, !lost);
        }
    }
}
const char *osdMultiMagMessage(uint32_t nowMs)
{
    const uint32_t elapsed = nowMs - lastDrawMs;
    if (count && wasVisible && elapsed <= 1000) {
        visibleMs += elapsed;
        if (visibleMs >= 3000) popEvent();
    }
    lastDrawMs = nowMs; wasVisible = false;
    if (!count && !fallback) return NULL;
    const unsigned number = (count ? events[0].slot : fallbackSlot) + 1;
    strcpy(message, "MAG ");
    unsigned offset = 4;
    if (number >= 10) message[offset++] = '0' + number / 10;
    message[offset++] = '0' + number % 10;
    strcpy(message + offset, count ? (events[0].restored ? " TRUST RESTORED" : " TRUST LOST") : " FALLBACK");
    if (count && fallback && events[0].slot == fallbackSlot) strcat(message, " FALLBACK");
    return message;
}
void osdMultiMagSetVisible(bool visible) { wasVisible = count && visible; }
#endif

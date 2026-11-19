/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <string.h>
#include "sensor_trust.h"

static void removeEvent(sensorTrustMonitor_t *monitor, unsigned index)
{
    monitor->count--;
    memmove(&monitor->events[index], &monitor->events[index + 1],
        (monitor->count - index) * sizeof(monitor->events[0]));
    if (index == 0) {
        monitor->visibleMs = 0;
        monitor->wasVisible = false;
    }
}

static void pushEvent(sensorTrustMonitor_t *monitor, unsigned source, bool restored)
{
    // Bounded memory under sustained repeated faults: retain the latest events.
    if (monitor->count == SENSOR_TRUST_EVENT_CAPACITY) {
        removeEvent(monitor, 0);
    }
    monitor->events[monitor->count++] = (sensorTrustEvent_t) {.source = source, .restored = restored};
}

void sensorTrustUpdate(sensorTrustMonitor_t *monitor, const sensorTrustInput_t inputs[SENSOR_TRUST_SOURCE_COUNT], uint32_t nowMs)
{
    for (unsigned i = 0; i < SENSOR_TRUST_SOURCE_COUNT; i++) {
        sensorTrustState_t *state = &monitor->source[i];
        if (!inputs[i].active) {
            memset(state, 0, sizeof(*state));
            for (unsigned j = 0; j < monitor->count;) {
                if (monitor->events[j].source == i) {
                    removeEvent(monitor, j);
                } else {
                    j++;
                }
            }
            continue;
        }
        const bool valid = inputs[i].valid && isfinite(inputs[i].confidence);
        const bool good = valid && inputs[i].confidence >= SENSOR_TRUST_RESTORED_THRESHOLD;
        const bool bad = !valid || inputs[i].confidence < SENSOR_TRUST_LOST_THRESHOLD;
        if (!state->known && good) {
            state->known = true;
            state->trusted = true; // Startup is a baseline, not a restoration event.
            state->pending = false;
            continue;
        }
        const bool changing = state->known && !state->trusted ? good : bad;
        if (!changing) {
            state->pending = false;
            continue;
        }
        if (!state->pending) {
            state->pending = true;
            state->pendingSinceMs = nowMs;
        } else if ((uint32_t)(nowMs - state->pendingSinceMs) >= SENSOR_TRUST_DEBOUNCE_MS) {
            state->trusted = state->known && !state->trusted;
            state->known = true;
            state->pending = false;
            pushEvent(monitor, i, state->trusted);
        }
    }
}

const sensorTrustEvent_t *sensorTrustPeek(sensorTrustMonitor_t *monitor, uint32_t nowMs)
{
    const uint32_t elapsed = nowMs - monitor->lastDrawMs;
    // Do not credit time when the OSD is hidden, stalled, or disconnected.
    if (monitor->count && monitor->wasVisible && elapsed <= 1000) {
        monitor->visibleMs += elapsed;
        if (monitor->visibleMs >= SENSOR_TRUST_DISPLAY_MS) {
            removeEvent(monitor, 0);
        }
    }
    monitor->lastDrawMs = nowMs;
    monitor->wasVisible = false;
    return monitor->count ? &monitor->events[0] : NULL;
}

void sensorTrustSetVisible(sensorTrustMonitor_t *monitor, bool visible)
{
    monitor->wasVisible = monitor->count && visible;
}

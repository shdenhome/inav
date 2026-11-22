/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <string.h>

#include "platform.h"

#include "sensors/compass_orientation.h"

#include "common/maths.h"
#include "common/vector.h"
#include "common/quaternion.h"

#if defined(USE_MAG) && defined(USE_MAG_CALIBRATION_ORIENTATION)

#define COMPASS_ORIENTATION_MIN_SAMPLES 30
#define COMPASS_ORIENTATION_LEGACY_CANDIDATES 16
#define COMPASS_ORIENTATION_CANDIDATE_COUNT 48

static compassOrientationContext_t legacyContext;

typedef struct {
    int16_t rollDD, pitchDD, yawDD;
} compassOrientationCandidate_t;

static const compassOrientationCandidate_t compassOrientationCandidates[COMPASS_ORIENTATION_CANDIDATE_COUNT] = {
    {   0, 0,    0}, {   0, 0,  900}, {   0, 0, 1800}, {   0, 0, 2700},
    {1800, 0,    0}, {1800, 0,  900}, {1800, 0, 1800}, {1800, 0, 2700},
    {   0, 0,  450}, {   0, 0, 1350}, {   0, 0, 2250}, {   0, 0, 3150},
    {1800, 0,  450}, {1800, 0, 1350}, {1800, 0, 2250}, {1800, 0, 3150},
    // Side-mounted multi-MAGs; the legacy wrapper still searches only its original 16.
    { 900,    0,    0}, { 900,    0,  450}, { 900,    0,  900}, { 900,    0, 1350}, { 900,    0, 1800}, { 900,    0, 2250}, { 900,    0, 2700}, { 900,    0, 3150},
    {2700,    0,    0}, {2700,    0,  450}, {2700,    0,  900}, {2700,    0, 1350}, {2700,    0, 1800}, {2700,    0, 2250}, {2700,    0, 2700}, {2700,    0, 3150},
    {   0,  900,    0}, {   0,  900,  450}, {   0,  900,  900}, {   0,  900, 1350}, {   0,  900, 1800}, {   0,  900, 2250}, {   0,  900, 2700}, {   0,  900, 3150},
    {   0, 2700,    0}, {   0, 2700,  450}, {   0, 2700,  900}, {   0, 2700, 1350}, {   0, 2700, 1800}, {   0, 2700, 2250}, {   0, 2700, 2700}, {   0, 2700, 3150},
};

// Running mean/variance of a 3-vector (Welford's algorithm, multivariate: m2
// accumulates squared distance to the running mean, summed over all 3 axes).
typedef struct {
    float mean[3];
    float m2;
    uint16_t n;
} compassOrientationStats_t;

static void statsReset(compassOrientationStats_t *s)
{
    memset(s, 0, sizeof(*s));
}

static void statsPush(compassOrientationStats_t *s, const fpVector3_t *sample)
{
    s->n++;
    float delta[3], delta2[3];
    for (int i = 0; i < 3; i++) {
        delta[i] = sample->v[i] - s->mean[i];
        s->mean[i] += delta[i] / s->n;
        delta2[i] = sample->v[i] - s->mean[i];
    }
    s->m2 += delta[0] * delta2[0] + delta[1] * delta2[1] + delta[2] * delta2[2];
}

static float statsVariance(const compassOrientationStats_t *s)
{
    return (s->n > 1) ? (s->m2 / (float)(s->n - 1)) : 0.0f;
}

static int16_t q15Encode(float x)
{
    return (int16_t)lrintf(constrainf(x, -1.0f, 1.0f) * 32767.0f);
}

static float q15Decode(int16_t x)
{
    return (float)x / 32767.0f;
}

void compassOrientationContextReset(compassOrientationContext_t *context)
{
    context->count = 0;
}

void compassOrientationContextPush(compassOrientationContext_t *context, const int16_t rawMag[3], const fpQuaternion_t *attitude)
{
    if (context->count >= COMPASS_ORIENTATION_MAX_SAMPLES) {
        return;
    }

    compassOrientationSample_t *s = &context->samples[context->count++];
    s->rawMag[0] = rawMag[0];
    s->rawMag[1] = rawMag[1];
    s->rawMag[2] = rawMag[2];
    s->q[0] = q15Encode(attitude->q0);
    s->q[1] = q15Encode(attitude->q1);
    s->q[2] = q15Encode(attitude->q2);
    s->q[3] = q15Encode(attitude->q3);
}

bool compassOrientationContextDetect(const compassOrientationContext_t *context, const float magZero[3], const int16_t magGain[3],
                               float confidenceMin,
                               int16_t *outRollDD, int16_t *outPitchDD, int16_t *outYawDD,
                               float *outConfidence)
{
    if (context->count < COMPASS_ORIENTATION_MIN_SAMPLES) {
        return false;
    }
    if (magGain[0] == 0 || magGain[1] == 0 || magGain[2] == 0) {
        return false;
    }

    if (context->extendedCandidates) {
        // Multi-MAG calibration must excite every sensor axis. A flat yaw spin
        // cannot identify a general mounting rotation even with many samples.
        for (unsigned axis = 0; axis < 3; axis++) {
            float minimum = INFINITY, maximum = -INFINITY;
            if (magGain[axis] <= 0 || !isfinite(magZero[axis])) return false;
            for (unsigned i = 0; i < context->count; i++) {
                const float value = (context->samples[i].rawMag[axis] - magZero[axis]) * 1024.0f / magGain[axis];
                minimum = fminf(minimum, value);
                maximum = fmaxf(maximum, value);
            }
            if (maximum - minimum < 512.0f) return false;
        }
    }

    int best = -1;
    int second = -1;
    float bestVar = 0.0f;
    float secondVar = 0.0f;
    float bestResidual = INFINITY;

    const int candidates = context->extendedCandidates ? COMPASS_ORIENTATION_CANDIDATE_COUNT : COMPASS_ORIENTATION_LEGACY_CANDIDATES;
    for (int c = 0; c < candidates; c++) {
        fp_angles_t angles = {
            .angles.roll  = DECIDEGREES_TO_RADIANS(compassOrientationCandidates[c].rollDD),
            .angles.pitch = DECIDEGREES_TO_RADIANS(compassOrientationCandidates[c].pitchDD),
            .angles.yaw   = DECIDEGREES_TO_RADIANS(compassOrientationCandidates[c].yawDD),
        };
        fpMat3_t rmat;
        rotationMatrixFromAngles(&rmat, &angles);

        compassOrientationStats_t stats;
        statsReset(&stats);
        compassOrientationStats_t directionStats;
        statsReset(&directionStats);

        for (uint16_t i = 0; i < context->count; i++) {
            const compassOrientationSample_t *s = &context->samples[i];

            const fpVector3_t calibrated = {
                .x = (s->rawMag[0] - magZero[0]) * 1024.0f / magGain[0],
                .y = (s->rawMag[1] - magZero[1]) * 1024.0f / magGain[1],
                .z = (s->rawMag[2] - magZero[2]) * 1024.0f / magGain[2],
            };
            const fpQuaternion_t q = {
                .q0 = q15Decode(s->q[0]), .q1 = q15Decode(s->q[1]),
                .q2 = q15Decode(s->q[2]), .q3 = q15Decode(s->q[3]),
            };

            fpVector3_t boardFrame, earthField;
            rotationMatrixRotateVector(&boardFrame, &calibrated, &rmat);
            quaternionRotateVectorInv(&earthField, &boardFrame, &q);
            statsPush(&stats, &earthField);
            if (context->extendedCandidates) {
                const float norm = sqrtf(sq(earthField.x) + sq(earthField.y) + sq(earthField.z));
                if (!isfinite(norm) || norm < 1e-6f) return false;
                fpVector3_t unit;
                for (unsigned axis = 0; axis < 3; axis++) unit.v[axis] = earthField.v[axis] / norm;
                statsPush(&directionStats, &unit);
            }
        }

        const float variance = statsVariance(&stats);
        if (best == -1 || variance < bestVar) {
            second = best;
            secondVar = bestVar;
            best = c;
            bestVar = variance;
            bestResidual = statsVariance(&directionStats);
        } else if (second == -1 || variance < secondVar) {
            second = c;
            secondVar = variance;
        }
    }

    if (context->extendedCandidates) {
        // A ratio alone can prefer a wrong discrete mount for an oblique sensor.
        // RMS chord residual of unit earth vectors must stay below 2*sin(5deg),
        // approximately 10deg RMS angular error. Reject a degenerate tied fit.
        const float maximumResidual = 0.030384494f;
        if (!isfinite(bestResidual) || bestResidual > maximumResidual || second < 0 ||
            !isfinite(bestVar) || !isfinite(secondVar) || secondVar < 1e-6f) {
            *outConfidence = 0;
            return false;
        }
    }

    (void)second;
    // bestVar can be ~0 for a mathematically-perfect fit; 1e-6f avoids a
    // near-zero/near-zero division producing a meaningless huge or NaN ratio.
    const float confidence = (bestVar < 1e-6f) ? confidenceMin : (secondVar / bestVar);
    *outConfidence = confidence;
    if (isnan(confidence) || isinf(confidence) || confidence < confidenceMin) {
        return false;
    }

    *outRollDD = compassOrientationCandidates[best].rollDD;
    *outPitchDD = compassOrientationCandidates[best].pitchDD;
    *outYawDD = compassOrientationCandidates[best].yawDD;
    return true;
}

void compassOrientationBufferReset(void)
{
    compassOrientationContextReset(&legacyContext);
}
void compassOrientationBufferPush(const int16_t rawMag[3], const fpQuaternion_t *attitude)
{
    compassOrientationContextPush(&legacyContext, rawMag, attitude);
}
bool compassOrientationDetect(const float magZero[3], const int16_t magGain[3],
    float confidenceMin, int16_t *outRollDD, int16_t *outPitchDD, int16_t *outYawDD, float *outConfidence)
{
    return compassOrientationContextDetect(&legacyContext, magZero, magGain,
        confidenceMin, outRollDD, outPitchDD, outYawDD, outConfidence);
}

#endif

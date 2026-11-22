/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#ifdef USE_MULTI_MAG
#if !defined(USE_MAG_CALIBRATION_ORIENTATION)
#error Multi magnetometer requires per-device calibration orientation support
#endif
#include <math.h>
#include <limits.h>
#include <string.h>
#include "common/utils.h"
#include "common/vector.h"
#include "common/quaternion.h"
#include "common/maths.h"
#include "config/parameter_group_ids.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/time.h"
#include "drivers/compass/compass.h"
#include "sensors/compass.h"
#include "sensors/compass_orientation.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/multi_mag.h"
#include "flight/imu.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "io/beeper.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_ak8975.h"
#include "drivers/compass/compass_hmc5883l.h"
#include "drivers/compass/compass_ist8308.h"
#include "drivers/compass/compass_ist8310.h"
#include "drivers/compass/compass_lis2mdl.h"
#include "drivers/compass/compass_lis3mdl.h"
#include "drivers/compass/compass_mag3110.h"
#include "drivers/compass/compass_mlx90393.h"
#include "drivers/compass/compass_qmc5883l.h"
#include "drivers/compass/compass_qmc5883p.h"
#include "drivers/compass/compass_vcm5883.h"

PG_REGISTER_WITH_RESET_TEMPLATE(multiMagConfig_t, multiMagConfig, PG_MULTI_MAG_CONFIG, 0);
PG_RESET_TEMPLATE(multiMagConfig_t, multiMagConfig,
    .enabled=SETTING_MULTI_MAG_ENABLED_DEFAULT,
    .maxAgeMs=SETTING_MULTI_MAG_MAX_AGE_MS_DEFAULT,
    .rejectMs=SETTING_MULTI_MAG_REJECT_MS_DEFAULT,
    .goodMs=SETTING_MULTI_MAG_GOOD_MS_DEFAULT,
    .recoveryMs=SETTING_MULTI_MAG_RECOVERY_MS_DEFAULT,
    .agreementDegrees=SETTING_MULTI_MAG_AGREEMENT_DEG_DEFAULT,
    .rejectDegrees=SETTING_MULTI_MAG_REJECT_DEG_DEFAULT,
    .fieldMinPercent=SETTING_MULTI_MAG_FIELD_MIN_PERCENT_DEFAULT,
    .fieldMaxPercent=SETTING_MULTI_MAG_FIELD_MAX_PERCENT_DEFAULT,
);

typedef struct multiMagDevice_s {
    magDev_t dev;
    busDevice_t bus;
    busDeviceDescriptor_t descriptor;
    uint32_t id, sampleAtUs, nextInitUs;
    uint8_t hardware;
    bool detected, readValid, readError, initialized, retryInit;
    int16_t raw[3];
    float aligned[3];
    fpMat3_t alignment;
    sensorCalibrationState_t calibration;
    compassOrientationContext_t orientation;
    float previous[3], peak[3];
    uint16_t sampleCount;
    uint8_t calibrationStatus, calibrationError;
} multiMagDevice_t;
static multiMagDevice_t devices[MULTI_MAG_MAX_SOURCES];
static magFusion_t fusion;
static magFusionSample_t samples[MULTI_MAG_MAX_SOURCES];
static fpMat3_t inverseTail;
static bool enabled, ready, calibrating, calibrationChanged;
static uint32_t calibrationStartedUs, calibrationRunId;
static unsigned finalizeSlot;
static uint16_t inventoryGeneration;
#ifdef MULTI_MAG_UNIT_TEST
// Host fixtures supply an ordinary array instead of MCU linker-section symbols.
extern const busDeviceDescriptor_t *multiMagTestRegistryStart;
extern const busDeviceDescriptor_t *multiMagTestRegistryEnd;
#define MAG_REGISTRY_START multiMagTestRegistryStart
#define MAG_REGISTRY_END multiMagTestRegistryEnd
#else
#define MAG_REGISTRY_START __busdev_registry_start
#define MAG_REGISTRY_END __busdev_registry_end
#endif
// Existing INAV I2C MAG endpoint catalog, plus LIS3MDL's documented second address.
// MATEKH743HD initializes I2C1/2: six addresses per bus fit all twelve stable slots.
static const uint8_t addresses[] = {0x0c,0x0d,0x0e,0x1c,0x1e,0x2c};
STATIC_ASSERT(ARRAYLEN(addresses)*2 == MULTI_MAG_MAX_SOURCES, multi_mag_endpoint_capacity);

static bool finiteVector(const float v[3]) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
static bool calibrationValid(unsigned slot)
{
    const multiMagCalibration_t *c=&multiMagConfig()->calibration[slot];
    if (!devices[slot].detected || c->id!=devices[slot].id || !(c->flags&MULTI_MAG_CAL_VALID)) return false;
    for (unsigned axis=0;axis<3;axis++) if (c->gain[axis]<=0) return false;
    return true;
}
static void refreshAlignment(unsigned slot)
{
    const multiMagCalibration_t *c=&multiMagConfig()->calibration[slot];
    fp_angles_t angles={.angles.roll=DECIDEGREES_TO_RADIANS(c->angles[0]),.angles.pitch=DECIDEGREES_TO_RADIANS(c->angles[1]),.angles.yaw=DECIDEGREES_TO_RADIANS(c->angles[2])};
    rotationMatrixFromAngles(&devices[slot].alignment,&angles);
}
static uint8_t typeForHardware(devHardwareType_e hw)
{
    switch(hw) {
    case DEVHW_QMC5883:return MAG_QMC5883; case DEVHW_QMC5883P:return MAG_QMC5883P;
    case DEVHW_HMC5883:return MAG_HMC5883; case DEVHW_AK8975:return MAG_AK8975;
    case DEVHW_AK8963:return MAG_AK8963; case DEVHW_MAG3110:return MAG_MAG3110;
    case DEVHW_IST8310_0:case DEVHW_IST8310_1:return MAG_IST8310;
    case DEVHW_IST8308:return MAG_IST8308; case DEVHW_LIS2MDL:return MAG_LIS2MDL;
    case DEVHW_LIS3MDL:return MAG_LIS3MDL;case DEVHW_VCM5883:return MAG_VCM5883;
    case DEVHW_MLX90393:return MAG_MLX90393;default:return MAG_NONE;
    }
}
static bool detectDevice(magDev_t *dev, uint8_t type)
{
    switch(type) {
#ifdef USE_MAG_QMC5883
    case MAG_QMC5883:return qmc5883Detect(dev);
#endif
#ifdef USE_MAG_QMC5883P
    case MAG_QMC5883P:return qmc5883pDetect(dev);
#endif
#ifdef USE_MAG_HMC5883
    case MAG_HMC5883:return hmc5883lDetect(dev);
#endif
#ifdef USE_MAG_AK8975
    case MAG_AK8975:return ak8975Detect(dev);
#endif
#ifdef USE_MAG_AK8963
    case MAG_AK8963:return ak8963Detect(dev);
#endif
#ifdef USE_MAG_MAG3110
    case MAG_MAG3110:return mag3110detect(dev);
#endif
#ifdef USE_MAG_IST8310
    case MAG_IST8310:return ist8310Detect(dev);
#endif
#ifdef USE_MAG_IST8308
    case MAG_IST8308:return ist8308Detect(dev);
#endif
#ifdef USE_MAG_LIS2MDL
    case MAG_LIS2MDL:return lis2mdlDetect(dev);
#endif
#ifdef USE_MAG_LIS3MDL
    case MAG_LIS3MDL:return lis3mdlDetect(dev);
#endif
#ifdef USE_MAG_VCM5883
    case MAG_VCM5883:return vcm5883Detect(dev);
#endif
#ifdef USE_MAG_MLX90393
    case MAG_MLX90393:return mlx90393Detect(dev);
#endif
    default:return false;
    }
}
static bool endpointClaimed(I2CDevice bus, uint8_t address)
{
#ifndef MULTI_MAG_UNIT_TEST
    for (const busDeviceDescriptor_t *d=MAG_REGISTRY_START;d<MAG_REGISTRY_END;d++) {
        const busDevice_t *b=d->devicePtr;
        if (d->busType==BUSTYPE_I2C && b && b->descriptorPtr==d && b->busType==BUSTYPE_I2C &&
            b->busdev.i2c.i2cBus==bus && b->busdev.i2c.address==address) return true;
    }
#else
    UNUSED(bus); UNUSED(address);
#endif
    return false;
}
// Pure probing: never modifies global sensor status or registry-owned device memory.
static bool probeEndpoint(unsigned slot, I2CDevice bus, uint8_t address)
{
    multiMagDevice_t *s=&devices[slot];
    // Preserve the existing AUTO preference when two compatible drivers share an ID.
    static const uint8_t order[]={MAG_QMC5883,MAG_QMC5883P,MAG_HMC5883,MAG_AK8975,MAG_AK8963,MAG_MAG3110,MAG_IST8310,MAG_IST8308,MAG_LIS2MDL,MAG_LIS3MDL,MAG_VCM5883,MAG_MLX90393};
    for (unsigned choice=0;choice<ARRAYLEN(order);choice++) {
        for (const busDeviceDescriptor_t *d=MAG_REGISTRY_START;d<MAG_REGISTRY_END;d++) {
            if (d->busType!=BUSTYPE_I2C || typeForHardware(d->devHwType)!=order[choice]) continue;
            if (d->busdev.i2c.address!=address && !(order[choice]==MAG_LIS3MDL && address==0x1c)) continue;
            memset(s,0,sizeof(*s));
            s->descriptor=*d; s->descriptor.devicePtr=&s->bus;
            s->descriptor.busdev.i2c.i2cBus=bus; s->descriptor.busdev.i2c.address=address;
            s->bus.descriptorPtr=&s->descriptor; s->bus.busType=BUSTYPE_I2C;
            s->bus.flags=d->flags; s->bus.param=d->param;
            s->bus.busdev.i2c.i2cBus=bus; s->bus.busdev.i2c.address=address;
            s->dev.busDev=&s->bus; s->dev.usePreconfiguredBus=true;
            if (!detectDevice(&s->dev,order[choice])) continue;
            // Identification owns this endpoint even when configuration fails.
            // Keep it visible and retry independently; never reinterpret it as
            // another chip merely because an initialization transaction NACKed.
            s->initialized=s->dev.init(&s->dev);
            s->readError=s->retryInit=!s->initialized;
            s->nextInitUs=(uint32_t)micros()+1000000;
            s->hardware=order[choice]; s->detected=true;
            s->id=((uint32_t)(bus+1)<<16)|((uint32_t)address<<8)|s->hardware;
            s->calibrationStatus=calibrationValid(slot)?MAG_CAL_OK:MAG_CAL_NEEDED;
            refreshAlignment(slot);
            return true;
        }
    }
    memset(s,0,sizeof(*s));
    return false;
}

bool multiMagEnabled(void) { return enabled; }
bool multiMagInit(void)
{
    enabled=multiMagConfig()->enabled; ready=calibrating=false; calibrationRunId=0;
    memset(devices,0,sizeof(devices)); memset(samples,0,sizeof(samples)); magFusionReset(&fusion);
    if (!enabled) return false;
    fp_angles_t tail={.angles.pitch=DECIDEGREES_TO_RADIANS(-900)};
    rotationMatrixFromAngles(&inverseTail,&tail);
    unsigned buses=0;
    for (int bus=0;bus<I2CDEV_COUNT;bus++) if (i2cIsInitialized(bus)) buses++;
    if (buses>2) { sensorsClear(SENSOR_MAG); return false; }
    inventoryGeneration=1;
    for (int bus=0;bus<I2CDEV_COUNT && bus<2;bus++) {
        if (!i2cIsInitialized(bus)) continue;
        for (unsigned address=0;address<ARRAYLEN(addresses);address++) {
            const unsigned slot=bus*ARRAYLEN(addresses)+address;
            if (endpointClaimed(bus,addresses[address])) continue;
            if (probeEndpoint(slot,bus,addresses[address]))
                inventoryGeneration=(inventoryGeneration*31u)^(devices[slot].id&0xffffu)^(devices[slot].id>>16);
        }
    }
    if (!inventoryGeneration) inventoryGeneration=1;
    requestedSensors[SENSOR_INDEX_MAG]=compassConfig()->mag_hardware;
    for (unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) if (devices[i].detected) {
        detectedSensors[SENSOR_INDEX_MAG]=devices[i].hardware; sensorsSet(SENSOR_MAG); return true;
    }
    detectedSensors[SENSOR_INDEX_MAG]=MAG_NONE; sensorsClear(SENSOR_MAG); return false;
}

static void startCalibration(uint32_t nowUs)
{
    calibrationRunId++; calibrating=true; calibrationChanged=false; calibrationStartedUs=nowUs; finalizeSlot=0;
    for (unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) if (devices[i].detected) {
        multiMagDevice_t *s=&devices[i];s->calibrationStatus=MAG_CAL_RUNNING;s->calibrationError=MAG_CAL_ERROR_NONE;
        s->sampleCount=0; memset(s->previous,0,sizeof(s->previous));memset(s->peak,0,sizeof(s->peak));
        sensorCalibrationResetState(&s->calibration);compassOrientationContextReset(&s->orientation);
        s->orientation.extendedCandidates=true;
    }
    DISABLE_STATE(CALIBRATE_MAG);beeper(BEEPER_ACTION_SUCCESS);
}
static void collectCalibration(multiMagDevice_t *s)
{
    float raw[3],difference=0,average=0;
    for (unsigned axis=0;axis<3;axis++) {
        raw[axis]=s->raw[axis];
        difference+=sq(raw[axis]-s->previous[axis]);average+=sq((raw[axis]+s->previous[axis])*0.5f);
        if (fabsf(raw[axis])>fabsf(s->peak[axis])) s->peak[axis]=raw[axis];
    }
    if (average>0.01f && difference/average>sq(0.14f) && isImuReady()) {
        sensorCalibrationPushSampleForOffsetCalculation(&s->calibration,raw);
        compassOrientationContextPush(&s->orientation,s->raw,&orientation);
        if (s->sampleCount<UINT16_MAX) s->sampleCount++;
        memcpy(s->previous,raw,sizeof(raw));
    }
}
static void finishOneCalibration(unsigned slot,uint32_t nowUs)
{
    multiMagDevice_t *s=&devices[slot];
    s->calibrationStatus=MAG_CAL_FAILED;
    if (!s->readValid || (uint32_t)(nowUs-s->sampleAtUs)>(uint32_t)multiMagConfig()->maxAgeMs*1000) {
        s->calibrationStatus=MAG_CAL_MISSING;s->calibrationError=MAG_CAL_ERROR_READ;return;
    }
    if (s->sampleCount<30) {s->calibrationError=MAG_CAL_ERROR_COVERAGE;return;}
    float zero[3];
    if (!sensorCalibrationSolveForOffset(&s->calibration,zero)||!finiteVector(zero)) {s->calibrationError=MAG_CAL_ERROR_SOLVER;return;}
    multiMagCalibration_t candidate={.id=s->id};
    for (unsigned axis=0;axis<3;axis++) {
        const float gain=fabsf(s->peak[axis]-zero[axis]);
        if (fabsf(zero[axis])>INT16_MAX || !isfinite(gain) || gain<1 || gain>INT16_MAX) {s->calibrationError=MAG_CAL_ERROR_GAIN;return;}
        candidate.zero[axis]=lrintf(zero[axis]);candidate.gain[axis]=lrintf(gain);
    }
    float confidence=0;
    if (!compassOrientationContextDetect(&s->orientation,zero,candidate.gain,COMPASS_ORIENTATION_CONFIDENCE_MIN,
        &candidate.angles[0],&candidate.angles[1],&candidate.angles[2],&confidence)) {s->calibrationError=MAG_CAL_ERROR_ORIENTATION;return;}
    candidate.flags=MULTI_MAG_CAL_VALID|(STATE(TAILSITTER)?MULTI_MAG_CAL_TAILSITTER:0);
    candidate.orientationQuality=isfinite(confidence)&&confidence>1?lrintf(fminf(100,(1-1/confidence)*100)):0;
    multiMagConfigMutable()->calibration[slot]=candidate;refreshAlignment(slot);
    s->calibrationStatus=MAG_CAL_OK;s->calibrationError=MAG_CAL_ERROR_NONE;calibrationChanged=true;
}

void multiMagUpdate(timeUs_t nowUs)
{
    if (!enabled) return;
    if (STATE(CALIBRATE_MAG)) startCalibration(nowUs);
    const uint32_t durationUs=(uint32_t)compassConfig()->magCalibrationTimeLimit*1000000u;
    const bool collecting=calibrating&&(uint32_t)(nowUs-calibrationStartedUs)<durationUs;
    const multiMagConfig_t *config=multiMagConfig();
    for (unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) {
        multiMagDevice_t *s=&devices[i];if (!s->detected) continue;
        // Driver initialization can block for tens of milliseconds. Never run it
        // armed; transient read errors first retry the existing measurement path.
        if ((!s->initialized || s->retryInit) && !ARMING_FLAG(ARMED) &&
            (int32_t)(nowUs-s->nextInitUs)>=0) {
            s->initialized=s->dev.init(&s->dev);s->retryInit=!s->initialized;s->nextInitUs=nowUs+1000000;
        }
        s->dev.readStatus=MAG_READ_NOT_READY;
        if (s->initialized) s->dev.read(&s->dev);
        if (s->initialized && s->dev.readStatus==MAG_READ_ERROR) {
            s->readValid=false;s->readError=true;
            if (!s->retryInit) {s->retryInit=true;s->nextInitUs=nowUs+1000000;}
        } else if (s->dev.readStatus==MAG_READ_READY) {
            memcpy(s->raw,s->dev.magADCRaw,sizeof(s->raw));s->sampleAtUs=nowUs;s->readError=false;s->retryInit=false;
            s->readValid=(s->raw[0]||s->raw[1]||s->raw[2]);
            for (unsigned axis=0;axis<3;axis++) if(s->raw[axis]==INT16_MIN||s->raw[axis]==INT16_MAX)s->readValid=false;
            if (collecting && s->readValid) collectCalibration(s);
        }
        magFusionSample_t *sample=&samples[i];sample->id=s->id;sample->timestampUs=s->sampleAtUs;
        sample->valid=s->readValid&&calibrationValid(i)&&!calibrating;
        sample->earthValid=false;
        if (calibrationValid(i)) {
            const multiMagCalibration_t *c=&config->calibration[i];fpVector3_t raw,body;
            for(unsigned axis=0;axis<3;axis++)raw.v[axis]=(s->raw[axis]-c->zero[axis])*1024.0f/c->gain[axis];
            rotationMatrixRotateVector(&body,&raw,&s->alignment);
            if(c->flags&MULTI_MAG_CAL_TAILSITTER)rotationMatrixRotateVector(&body,&body,&inverseTail);
            applyTailSitterAlignment(&body);
            memcpy(s->aligned,body.v,sizeof(s->aligned));memcpy(sample->field,body.v,sizeof(sample->field));
            if(isImuReady()&&gyroIsCalibrationComplete()&&getHwGyroStatus()==HW_SENSOR_OK) {
                fpVector3_t earth;quaternionRotateVectorInv(&earth,&body,&orientation);
                memcpy(sample->earthField,earth.v,sizeof(sample->earthField));sample->earthValid=finiteVector(earth.v);
            }
        }
    }
    if (calibrating&&!collecting) {
        // At most one bounded orientation search per compass task, not twelve in one PID gap.
        while(finalizeSlot<MULTI_MAG_MAX_SOURCES&&!devices[finalizeSlot].detected)finalizeSlot++;
        if(finalizeSlot<MULTI_MAG_MAX_SOURCES)finishOneCalibration(finalizeSlot++,nowUs);
        else {
            calibrating=false;magFusionReset(&fusion);
            if(calibrationChanged)saveConfigAndNotify();
        }
    }
    const magFusionConfig_t policy={.maxAgeUs=(uint32_t)config->maxAgeMs*1000,.rejectUs=(uint32_t)config->rejectMs*1000,
        .goodUs=(uint32_t)config->goodMs*1000,.recoverySeconds=config->recoveryMs*0.001f,
        .agreementDegrees=config->agreementDegrees,.rejectDegrees=config->rejectDegrees,
        .fieldMin=config->fieldMinPercent*10.24f,.fieldMax=config->fieldMaxPercent*10.24f};
    if (magFusionUpdate(&fusion,samples,nowUs,&policy)) {
        memcpy(mag.magADC,fusion.output,sizeof(mag.magADC));memcpy(mag.magADCUnaligned,fusion.output,sizeof(mag.magADCUnaligned));ready=true;
    } else {ready=false;memset(mag.magADC,0,sizeof(mag.magADC));memset(mag.magADCUnaligned,0,sizeof(mag.magADCUnaligned));}
    if(multiMagIsCalibrated())ENABLE_STATE(COMPASS_CALIBRATED);else DISABLE_STATE(COMPASS_CALIBRATED);
}
bool multiMagIsHealthy(void){return enabled&&fusion.healthy&&!calibrating;}
bool multiMagIsReady(void){return ready;}
bool multiMagIsCalibrating(void){return calibrating;}
bool multiMagIsCalibrated(void){for(unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++)if(calibrationValid(i))return true;return false;}
bool multiMagDetected(unsigned slot){return slot<MULTI_MAG_MAX_SOURCES&&devices[slot].detected;}
uint32_t multiMagReferenceId(void){return fusion.referenceId;}
uint32_t multiMagFallbackId(void){return fusion.fallbackId;}
uint32_t multiMagCalibrationRunId(void){return calibrationRunId;}
unsigned multiMagCount(void){unsigned n=0;for(unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++)n+=devices[i].detected;return n;}

bool multiMagCaptureSource(unsigned slot,uint32_t nowUs,multiMagSourceState_t *state)
{
    if(!state||!multiMagDetected(slot))return false;
    const multiMagDevice_t *s=&devices[slot];const magFusionMember_t *m=&fusion.member[slot];const multiMagCalibration_t *c=&multiMagConfig()->calibration[slot];
    memset(state,0,sizeof(*state));state->id=s->id;state->slot=slot;state->hardware=s->hardware;
    state->bus=s->descriptor.busdev.i2c.i2cBus+1;state->address=s->descriptor.busdev.i2c.address;
    state->ageUs=s->readValid?(uint32_t)(nowUs-s->sampleAtUs):UINT32_MAX;
    state->flags=MULTI_MAG_DETECTED|(s->readValid?MULTI_MAG_READABLE:0)|(s->readError?MULTI_MAG_READ_ERROR:0);
    if(s->readValid&&state->ageUs<=(uint32_t)multiMagConfig()->maxAgeMs*1000)state->flags|=MULTI_MAG_FRESH;
    if(calibrationValid(slot))state->flags|=MULTI_MAG_CALIBRATED|MULTI_MAG_ORIENTATION_VALID;
    if(calibrating&&s->calibrationStatus==MAG_CAL_RUNNING)state->flags|=MULTI_MAG_CALIBRATING;
    if(m->contributing)state->flags|=MULTI_MAG_CONTRIBUTING;
    if(fusion.fallbackId==s->id)state->flags|=MULTI_MAG_FALLBACK;
    if(m->confidence>0){state->flags|=MULTI_MAG_POSITIVE_TRUST;state->confidence=fmaxf(1,ceilf(fminf(1,m->confidence)*1000));}
    if(s->calibrationStatus==MAG_CAL_FAILED||s->calibrationStatus==MAG_CAL_MISSING)state->flags|=MULTI_MAG_CALIBRATION_FAILED;
    memcpy(state->raw,s->raw,sizeof(state->raw));memcpy(state->aligned,s->aligned,sizeof(state->aligned));
    if(c->id==s->id){memcpy(state->angles,c->angles,sizeof(state->angles));state->orientationQuality=c->orientationQuality;}
    state->calibrationStatus=s->calibrationStatus;state->failureCode=s->calibrationError;state->sampleCount=s->sampleCount;
    return true;
}
static int32_t integerField(float v){return !isfinite(v)?0:v>=2147483520.0f?INT32_MAX:v<=-2147483648.0f?INT32_MIN:lrintf(v);}
void multiMagSerializeState(sbuf_t *dst,uint8_t cursor,uint16_t expectedGeneration)
{
    UNUSED(expectedGeneration); // Client detects a generation mismatch and restarts its pass.
    const uint32_t nowUs=micros();unsigned total=multiMagCount();if(cursor>total)cursor=total;
    const unsigned count=MIN((unsigned)MULTI_MAG_PAGE_SIZE,total-cursor);
    sbufWriteU8(dst,MULTI_MAG_STATE_VERSION);sbufWriteU8(dst,(enabled?1:0)|(calibrating?2:0)|(multiMagIsHealthy()?4:0)|(fusion.fallbackId?8:0));
    sbufWriteU8(dst,total);sbufWriteU8(dst,cursor);sbufWriteU8(dst,count);sbufWriteU8(dst,cursor+count<total?cursor+count:255);
    sbufWriteU16(dst,inventoryGeneration);sbufWriteU32(dst,nowUs);sbufWriteU32(dst,fusion.referenceId);sbufWriteU32(dst,fusion.fallbackId);sbufWriteU32(dst,calibrationRunId);
    unsigned index=0;
    for(unsigned slot=0;slot<MULTI_MAG_MAX_SOURCES;slot++)if(devices[slot].detected){
        if(index++<cursor||index>cursor+count)continue;
        multiMagSourceState_t s;multiMagCaptureSource(slot,nowUs,&s);
        sbufWriteU32(dst,s.id);sbufWriteU8(dst,s.slot);sbufWriteU8(dst,s.hardware);sbufWriteU8(dst,s.bus);sbufWriteU8(dst,s.address);
        sbufWriteU16(dst,s.flags);sbufWriteU16(dst,s.confidence);sbufWriteU32(dst,s.ageUs);
        for(unsigned a=0;a<3;a++) { sbufWriteU16(dst,s.raw[a]); }
        for(unsigned a=0;a<3;a++) { sbufWriteU32(dst,integerField(s.aligned[a])); }
        sbufWriteU8(dst,s.calibrationStatus);sbufWriteU8(dst,s.orientationQuality);for(unsigned a=0;a<3;a++)sbufWriteU16(dst,s.angles[a]);
        sbufWriteU16(dst,s.sampleCount);sbufWriteU8(dst,s.failureCode);sbufWriteU8(dst,0);sbufWriteU8(dst,0);sbufWriteU8(dst,0);
    }
}
#endif

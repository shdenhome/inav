/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "sensors/multi_sensor.h"
#include "sensors/multi_acc.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "fc/runtime_config.h"
}
#include "gtest/gtest.h"
namespace {
uint32_t clockUs;
bool present[3], ack[3], frozen[3];
uint32_t frozenAt[3];
uint8_t tagOrder[3];
float pose[3], spike[3];
unsigned saved;
const float bias[3][3]={{100,-120,80},{-50,30,10},{150,80,-25}};
const float gain[3][3]={{1.03f,0.97f,1.02f},{1.01f,1.02f,0.99f},{0.98f,1.03f,1.01f}};
}
extern "C" {
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
void pgResetFn_accelerometerConfig(accelerometerConfig_t *);
gyroConfig_t gyroConfig_System;
uint8_t requestedSensors[SENSOR_INDEX_COUNT];
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
void sensorsSet(uint32_t) {}
uint32_t stateFlags;
uint32_t armingFlags;
timeUs_t micros(void) { return clockUs; }
timeMs_t millis(void) { return clockUs / 1000; }
uint32_t gyroGetImuSampleTime(unsigned slot) { return frozen[slot] ? frozenAt[slot] : clockUs; }
uint8_t gyroGetImuTag(unsigned slot) { return slot<3 && present[tagOrder[slot]] ? tagOrder[slot] : UINT8_MAX; }
void saveConfigAndNotify(void) { saved++; accSetCalibrationValues(); }
void beeperConfirmationBeeps(uint8_t) {}
static void initFake(accDev_t *dev) { dev->acc_1G=dev->imuSensorToUse==1 ? 4096 : 2048; }
static bool readFake(accDev_t *dev) {
    const unsigned tag=dev->imuSensorToUse;
    if (!ack[tag]) return false;
    float body[3];
    for (unsigned axis=0;axis<3;axis++) {
        body[axis]=(pose[axis]*4096/gain[tag][axis]+bias[tag][axis]+spike[tag])*dev->acc_1G/4096;
    }
    if (tag==2) {
        dev->ADCRaw[0]=body[1]; dev->ADCRaw[1]=body[0]; dev->ADCRaw[2]=-body[2];
    } else {
        dev->ADCRaw[0]=-body[0]; dev->ADCRaw[1]=body[1]; dev->ADCRaw[2]=-body[2];
    }
    return true;
}
static bool detect(accDev_t *dev, bool icm) {
    const unsigned tag=dev->imuSensorToUse;
    if (tag>=3 || !present[tag] || ((tag!=1)!=icm)) return false;
    dev->initFn=initFake; dev->readFn=readFake;
    dev->accAlign=tag==2 ? CW90_DEG_FLIP : CW0_DEG_FLIP;
    return true;
}
bool fakeAccDetect(accDev_t *dev) { return detect(dev,dev->imuSensorToUse!=1); }
bool icm42605AccDetect(accDev_t *dev) { return detect(dev,true); }
bool mpu6500AccDetect(accDev_t *dev) { return detect(dev,false); }
}
class MultiAccTest : public ::testing::Test {
protected:
    float output[3]{};
    void SetUp() override {
        clockUs=1000; saved=0; stateFlags=armingFlags=0;
        memset(pose,0,sizeof(pose)); memset(spike,0,sizeof(spike));
        memset(frozen,0,sizeof(frozen));
        pose[2]=1;
        for (unsigned i=0;i<3;i++) { present[i]=ack[i]=true; tagOrder[i]=i; }
        *multiSensorConfigMutable()=pgResetTemplate_multiSensorConfig;
        multiSensorConfigMutable()->imuEnabled=true;
        multiSensorLatchImuMode();
        pgResetFn_accelerometerConfig(accelerometerConfigMutable());
        accelerometerConfigMutable()->acc_lpf_hz=0;
        gyroConfigMutable()->gyro_to_use=0;
        ASSERT_TRUE(accInit(1000));
    }
    void run(unsigned count) {
        for (unsigned i=0;i<count;i++) { clockUs+=1000; multiAccUpdate(output); }
    }
    void calibrate() {
        const unsigned axes[6]={2,2,0,0,1,1};
        for (unsigned face=0;face<6;face++) {
            memset(pose,0,sizeof(pose)); pose[axes[face]]=face%2 ? -1 : 1;
            run(5); multiAccStartCalibration();
            ASSERT_FALSE(multiAccCalibrationComplete());
            run(510);
            ASSERT_TRUE(multiAccCalibrationComplete());
        }
        ASSERT_TRUE(multiAccCalibrationValid()); ASSERT_EQ(saved,1u);
    }
};
TEST_F(MultiAccTest, UncalibratedDevicesDoNotFeedAttitudeEstimator) {
    run(10); EXPECT_FALSE(multiAccFusion.healthy); EXPECT_FALSE(multiAccCalibrationValid());
}
TEST_F(MultiAccTest, SixPositionsCalibrateAllPhysicalDevicesAndPersistByTag) {
    calibrate();
    EXPECT_EQ(multiSensorConfig()->accCalibratedMask,7);
    for (unsigned tag=0;tag<3;tag++) for (unsigned axis=0;axis<3;axis++) {
        EXPECT_NEAR(multiSensorConfig()->accZero[tag][axis],bias[tag][axis],2);
        EXPECT_NEAR(multiSensorConfig()->accGain[tag][axis]/4096.0f,gain[tag][axis],0.001);
    }
    pose[0]=0.25f; pose[1]=-0.5f; pose[2]=0.75f;
    run(10); ASSERT_TRUE(multiAccFusion.healthy);
    for (unsigned axis=0;axis<3;axis++) EXPECT_NEAR(output[axis],pose[axis],0.001);
    EXPECT_EQ(multiAccFusion.contributors,3);
}
TEST_F(MultiAccTest, CalibrationPersistsAcrossDeviceReinitialization) {
    calibrate(); ASSERT_TRUE(multiAccInit()); EXPECT_TRUE(multiAccCalibrationValid());
    run(10); EXPECT_TRUE(multiAccFusion.healthy);
}
TEST_F(MultiAccTest, NackAndOutlierDoNotCreateZeroMeasurements) {
    calibrate(); run(10);
    spike[2]=40960; run(10);
    EXPECT_LT(multiAccFusion.member[2].confidence,0.01f);
    EXPECT_NEAR(output[1],-1,0.01);
    ack[0]=false; run(10);
    EXPECT_FLOAT_EQ(multiAccFusion.member[0].confidence,0);
    ack[1]=ack[2]=false; run(10); EXPECT_FALSE(multiAccFusion.healthy);
}
TEST_F(MultiAccTest, CalibrationCannotStartWithoutFreshReadFromEveryDetectedDevice) {
    run(5); ack[1]=false; run(5); multiAccStartCalibration();
    EXPECT_TRUE(multiAccCalibrationComplete()); EXPECT_EQ(saved,0u);
}
TEST_F(MultiAccTest, MissingDeviceDoesNotStealAnotherTagCalibration) {
    present[1]=false; ASSERT_TRUE(multiAccInit()); calibrate();
    EXPECT_EQ(multiSensorConfig()->accCalibratedMask,5);
    EXPECT_EQ(multiSensorConfig()->accZero[1][0],0);
    run(5); EXPECT_EQ(multiAccFusion.contributors,2);
}

TEST_F(MultiAccTest, FullAccelerometerPipelineAppliesAlignmentOnceAndUsesPerDeviceCalibration) {
    calibrate();
    for (unsigned axis=0;axis<3;axis++) {
        accelerometerConfigMutable()->accZero.raw[axis]=1000;
        accelerometerConfigMutable()->accGain.raw[axis]=8192;
    }
    ASSERT_TRUE(accInit(1000));
    pose[0]=0.25f; pose[1]=-0.5f; pose[2]=0.75f;
    for (unsigned i=0;i<10;i++) { clockUs+=1000; accUpdate(); }
    ASSERT_TRUE(accIsHealthy());
    for (unsigned axis=0;axis<3;axis++) EXPECT_NEAR(acc.accADCf[axis],pose[axis],0.001);
}

TEST_F(MultiAccTest, CachedMpuAccelerationIsNotRetimestampedAsFresh) {
    calibrate(); run(10);
    frozen[1]=true; frozenAt[1]=clockUs;
    run(20);
    EXPECT_FALSE(multiAccSamples[1].valid);
    EXPECT_FLOAT_EQ(multiAccFusion.member[1].confidence,0);
    EXPECT_EQ(multiAccFusion.contributors,2);
}

TEST_F(MultiAccTest, SavedCalibrationFollowsPhysicalTagWhenPrimarySlotChanges) {
    calibrate();
    tagOrder[0]=2; tagOrder[1]=0; tagOrder[2]=1;
    gyroConfigMutable()->gyro_to_use=2;
    ASSERT_TRUE(accInit(1000));
    pose[0]=0.25f; pose[1]=-0.5f; pose[2]=0.75f;
    for (unsigned i=0;i<10;i++) { clockUs+=1000; accUpdate(); }
    ASSERT_TRUE(accIsHealthy());
    for (unsigned axis=0;axis<3;axis++) EXPECT_NEAR(acc.accADCf[axis],pose[axis],0.001);
}

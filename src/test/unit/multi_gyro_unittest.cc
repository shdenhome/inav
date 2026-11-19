/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "common/log.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "sensors/gyro.h"
#include "sensors/multi_sensor.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "scheduler/scheduler.h"
#include "fc/runtime_config.h"
#include "fc/config.h"
}
#include "gtest/gtest.h"
namespace {
uint32_t clockUs;
bool present[3], ack[3], initOk[3];
float rates[3][3];
unsigned reads[3];
uint32_t periods[3];
}
extern "C" {
extern const gyroConfig_t pgResetTemplate_gyroConfig;
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
uint8_t requestedSensors[SENSOR_INDEX_COUNT];
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
uint32_t armingFlags;
uint32_t stateFlags;
timeUs_t micros(void) { return clockUs; }
timeMs_t millis(void) { return clockUs / 1000; }
void sensorsSet(uint32_t) {}
void sensorsClear(uint32_t) {}
void schedulerResetTaskStatistics(cfTaskId_e) {}
void busDeviceDeInit(busDevice_t *) {}
uint32_t getLooptime(void) { return 1000; }
uint32_t getGyroLooptime(void) { return 1000; }
void _logf(logTopic_e, unsigned, const char *, ...) {}
static void initFake(gyroDev_t *dev) { dev->sampleRateIntervalUs=initOk[dev->imuSensorToUse] ? periods[dev->imuSensorToUse] : 0; }
static bool readFake(gyroDev_t *dev) {
    const unsigned tag=dev->imuSensorToUse;
    reads[tag]++;
    if (!ack[tag]) return false;
    float body[3];
    for (unsigned axis=0;axis<3;axis++) body[axis]=rates[tag][axis]/dev->scale;
    if (tag==1) {
        dev->gyroADCRaw[0]=-body[1]; dev->gyroADCRaw[1]=body[0]; dev->gyroADCRaw[2]=body[2];
    } else if (tag==2) {
        dev->gyroADCRaw[0]=-body[0]; dev->gyroADCRaw[1]=-body[1]; dev->gyroADCRaw[2]=body[2];
    } else memcpy(dev->gyroADCRaw,body,sizeof(body));
    return true;
}
bool fakeGyroDetect(gyroDev_t *dev) {
    const unsigned tag=dev->imuSensorToUse;
    if (tag>=3 || !present[tag]) return false;
    dev->initFn=initFake; dev->readFn=readFake;
    dev->scale=tag==1 ? 1.0f/32.8f : 1.0f/16.4f;
    dev->gyroAlign=tag==1 ? CW90_DEG : tag==2 ? CW180_DEG : CW0_DEG;
    return true;
}
}
class MultiGyroTest : public ::testing::Test {
protected:
    void SetUp() override {
        clockUs=1000; armingFlags=stateFlags=0;
        memset(rates,0,sizeof(rates)); memset(reads,0,sizeof(reads));
        for (unsigned i=0;i<3;i++) { present[i]=ack[i]=initOk[i]=true; periods[i]=1000; }
        *gyroConfigMutable()=pgResetTemplate_gyroConfig;
        gyroConfigMutable()->gyro_anti_aliasing_lpf_hz=0;
        gyroConfigMutable()->gyro_to_use=0;
        *multiSensorConfigMutable()=pgResetTemplate_multiSensorConfig;
        multiSensorConfigMutable()->imuEnabled=true;
    }
    void run(unsigned iterations) {
        for (unsigned i=0;i<iterations;i++) { clockUs+=1000; gyroUpdate(); }
    }
    void start() {
        ASSERT_TRUE(gyroInit()); gyroStartCalibration(); run(2200);
        ASSERT_TRUE(gyroIsCalibrationComplete());
    }
};
TEST_F(MultiGyroTest, ThreeDevicesCalibrateAndAveragePhysicalBodyRates) {
    start();
    for (unsigned tag=0;tag<3;tag++) { rates[tag][0]=10+tag*2; rates[tag][1]=-20; rates[tag][2]=30; }
    run(10);
    EXPECT_EQ(multiGyroFusion.contributors,3);
    EXPECT_NEAR(gyro.gyroADCf[0],12,0.001);
    EXPECT_NEAR(gyro.gyroADCf[1],-20,0.001);
    EXPECT_NEAR(gyro.gyroADCf[2],30,0.001);
    for (unsigned tag=0;tag<3;tag++) EXPECT_GT(reads[tag],2200u);
}
TEST_F(MultiGyroTest, MissingPreferredDeviceDoesNotHideRemainingImus) {
    present[0]=false; start();
    EXPECT_EQ(gyroGetImuTag(0),1);
    run(10); EXPECT_EQ(multiGyroFusion.contributors,2);
}
TEST_F(MultiGyroTest, IndependentZeroCalibrationRemovesIndividualBiases) {
    for (unsigned tag=0;tag<3;tag++) rates[tag][0]=(int)tag-1;
    start(); run(10);
    EXPECT_NEAR(gyro.gyroADCf[0],0,0.001);
    EXPECT_EQ(multiGyroFusion.contributors,3);
}
TEST_F(MultiGyroTest, IsolatedSpikeAndReadFailureReduceContribution) {
    start(); rates[2][0]=10000; run(10);
    EXPECT_LT(multiGyroFusion.member[2].confidence,0.001f);
    EXPECT_LT(gyro.gyroADCf[0],1);
    ack[0]=false; run(10);
    EXPECT_FLOAT_EQ(multiGyroFusion.member[0].confidence,0);
    ack[1]=ack[2]=false; run(10);
    EXPECT_FALSE(multiGyroFusion.healthy);
}
TEST_F(MultiGyroTest, ModeChangeIsLatchedUntilReinitialization) {
    start(); multiSensorConfigMutable()->imuEnabled=false;
    EXPECT_TRUE(multiSensorImuEnabled());
    run(10); EXPECT_EQ(multiGyroFusion.contributors,3);
}
TEST_F(MultiGyroTest, DisabledModeKeepsSingleDeviceReadPath) {
    multiSensorConfigMutable()->imuEnabled=false;
    ASSERT_TRUE(gyroInit()); gyroStartCalibration(); run(2200);
    EXPECT_GT(reads[0],0u); EXPECT_EQ(reads[1],0u); EXPECT_EQ(reads[2],0u);
}

TEST_F(MultiGyroTest, MixedRatesDoNotOverweightTheFasterDevice) {
    periods[1]=2000; start();
    rates[0][0]=10; rates[1][0]=12; rates[2][0]=14;
    run(20);
    EXPECT_NEAR(gyro.gyroADCf[0],12,0.001);
    EXPECT_EQ(multiGyroFusion.contributors,3);
    EXPECT_GT(reads[0],reads[1]*1.9);
}
TEST_F(MultiGyroTest, AllMissingImusCannotBeReportedAsHealthyOrCalibrated) {
    for (unsigned tag=0;tag<3;tag++) present[tag]=false;
    gyroInit(); gyroStartCalibration(); run(10);
    EXPECT_FALSE(gyro.initialized); EXPECT_FALSE(gyroIsCalibrationComplete());
    EXPECT_FALSE(multiGyroFusion.healthy); EXPECT_EQ(gyroGetImuTag(0),UINT8_MAX);
}

TEST_F(MultiGyroTest, PreferredTagChangesSlotOrderWithoutChangingBodyMean) {
    gyroConfigMutable()->gyro_to_use=2; start();
    EXPECT_EQ(gyroGetImuTag(0),2); EXPECT_EQ(gyroGetImuTag(1),0); EXPECT_EQ(gyroGetImuTag(2),1);
    for (unsigned tag=0;tag<3;tag++) rates[tag][0]=10+2*tag;
    run(10); EXPECT_NEAR(gyro.gyroADCf[0],12,0.001);
}

TEST_F(MultiGyroTest, FailedInitializationIsExcludedAndAnotherImuCanBecomePrimary) {
    initOk[0]=false; start();
    EXPECT_EQ(gyroGetImuTag(0),1); EXPECT_EQ(gyroGetImuTag(1),2);
    EXPECT_EQ(multiGyroFusion.contributors,2);
}

TEST_F(MultiGyroTest, SlowPreferredImuDoesNotThrottleAcquisitionOfFasterMembers) {
    periods[1]=2000; gyroConfigMutable()->gyro_to_use=1; start();
    EXPECT_EQ(gyroGetImuTag(0),1);
    EXPECT_EQ(gyro.targetLooptime,1000u);
    EXPECT_GT(reads[0],reads[1]*1.9);
    EXPECT_GT(reads[2],reads[1]*1.9);
}

TEST_F(MultiGyroTest, LastReadingImuFeedsControllerDespiteConfidenceLoss) {
    start(); ack[0] = ack[1] = false; rates[2][0] = 1000;
    run(1);
    ASSERT_TRUE(multiGyroFusion.healthy); ASSERT_TRUE(multiGyroFusion.fallbackActive);
    EXPECT_EQ(multiGyroFusion.contributors, 1); EXPECT_EQ(multiGyroFusion.contributorMask, 4);
    EXPECT_LT(multiGyroFusion.member[2].confidence, SENSOR_FUSION_MIN_CONFIDENCE);
    EXPECT_NEAR(gyro.gyroADCf[0], 1000, 0.1);
    ack[2] = false; run(1);
    EXPECT_FALSE(multiGyroFusion.healthy); EXPECT_EQ(multiGyroFusion.contributors, 0);
}

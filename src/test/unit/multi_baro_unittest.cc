/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "common/log.h"
#include "drivers/time.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"
#include "sensors/multi_sensor.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
}
#include "gtest/gtest.h"

namespace {
uint32_t clockUs;
uint32_t sensorMask;
busDevice_t buses[2];
struct FakeBaro {
    bool present = true;
    bool ack = true;
    bool pressureConversion = false;
    uint32_t readyAt = 0;
    unsigned temperatureReads = 0;
    unsigned pressureReads = 0;
    int32_t pressure = 100000;
};
FakeBaro chips[2];
unsigned index(baroDev_t *dev) { return dev->busDev - buses; }
}
extern "C" {
extern const multiSensorConfig_t pgResetTemplate_multiSensorConfig;
extern const pgRegistry_t multiSensorConfig_Registry;
uint8_t requestedSensors[SENSOR_INDEX_COUNT];
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
uint32_t armingFlags;
timeUs_t micros(void) { return clockUs; }
timeMs_t millis(void) { return clockUs / 1000; }
void sensorsSet(uint32_t mask) { sensorMask |= mask; }
void sensorsClear(uint32_t mask) { sensorMask &= ~mask; }
bool sensors(uint32_t mask) { return (sensorMask & mask) != 0; }
float applySensorTempCompensation(int16_t, float value, sensorIndex_e) { return value; }
void _logf(logTopic_e, unsigned, const char *, ...) {}
static bool startTemperature(baroDev_t *dev) {
    auto &chip = chips[index(dev)];
    chip.pressureConversion = false;
    chip.readyAt = clockUs + dev->ut_delay;
    return chip.ack;
}
static bool startPressure(baroDev_t *dev) {
    auto &chip = chips[index(dev)];
    chip.pressureConversion = true;
    chip.readyAt = clockUs + dev->up_delay;
    return chip.ack;
}
static bool getTemperature(baroDev_t *dev) {
    auto &chip = chips[index(dev)];
    EXPECT_FALSE(chip.pressureConversion);
    EXPECT_GE((int32_t)(clockUs-chip.readyAt),0);
    chip.temperatureReads++;
    return chip.ack;
}
static bool getPressure(baroDev_t *dev) {
    auto &chip = chips[index(dev)];
    EXPECT_TRUE(chip.pressureConversion);
    EXPECT_GE((int32_t)(clockUs-chip.readyAt),0);
    chip.pressureReads++;
    return chip.ack;
}
static bool calculate(baroDev_t *dev, int32_t *pressure, int32_t *temperature) {
    *pressure = chips[index(dev)].pressure;
    *temperature = 2000;
    return chips[index(dev)].ack;
}
static bool detect(baroDev_t *dev, unsigned i) {
    if (!chips[i].present) return false;
    dev->busDev = &buses[i];
    dev->ut_delay = i == 0 ? 10000 : 30000;
    dev->up_delay = i == 0 ? 20000 : 40000;
    dev->start_ut = startTemperature; dev->get_ut = getTemperature;
    dev->start_up = startPressure; dev->get_up = getPressure;
    dev->calculate = calculate;
    return true;
}
bool ms5611Detect(baroDev_t *dev) { return detect(dev,0); }
bool spl06Detect(baroDev_t *dev) { return detect(dev,1); }
}

class MultiBaroTest : public ::testing::Test {
protected:
    void SetUp() override {
        clockUs = 1000; sensorMask = 0; armingFlags = 0;
        chips[0] = FakeBaro{}; chips[1] = FakeBaro{};
        chips[1].pressure = 100020;
        *multiSensorConfigMutable() = pgResetTemplate_multiSensorConfig;
        multiSensorConfigMutable()->baroEnabled = true;
        multiSensorConfigMutable()->baroDeviationPa = 100;
        multiSensorConfigMutable()->recoveryMs = 5000;
        multiSensorConfigMutable()->baroMaxAgeMs = 200;
        memset(&baro,0,sizeof(baro));
        memset(detectedSensors,0,sizeof(detectedSensors));
    }
    void run(unsigned iterations) {
        for (unsigned i=0;i<iterations;i++) {
            uint32_t wait=baroUpdate();
            ASSERT_GE(wait,1000u);
            clockUs+=wait;
        }
    }
};

TEST_F(MultiBaroTest, HardwareInventoryReportsDetectedChipsRatherThanPressureHealth) {
    ASSERT_TRUE(baroInit());
    EXPECT_EQ(baroGetSensorHardware(0), BARO_MS5611);
    EXPECT_EQ(baroGetSensorHardware(1), BARO_SPL06);
    EXPECT_EQ(baroGetSensorHardware(2), BARO_NONE);
    chips[1].ack = false;
    run(30);
    EXPECT_EQ(baroGetSensorHardware(1), BARO_SPL06);
    chips[1].present = false;
    ASSERT_TRUE(baroInit());
    EXPECT_EQ(baroGetSensorHardware(1), BARO_NONE);
    multiSensorConfigMutable()->baroEnabled = false;
    multiSensorLatchBaroMode();
    EXPECT_EQ(baroGetSensorHardware(0), BARO_NONE);
}
TEST_F(MultiBaroTest, IndependentConversionDelaysAndAveraging) {
    ASSERT_TRUE(baroInit());
    EXPECT_FALSE(baroIsHealthy());
    run(100);
    EXPECT_GT(chips[0].pressureReads, chips[1].pressureReads);
    EXPECT_GT(chips[1].pressureReads, 0u);
    EXPECT_TRUE(baroIsHealthy());
    EXPECT_EQ(multiBaroFusion.contributors,2);
    EXPECT_EQ(baro.baroPressure,100010);
}
TEST_F(MultiBaroTest, EitherPhysicalSensorCanBeAbsent) {
    for (unsigned missing=0;missing<2;missing++) {
        chips[0].present = chips[1].present = true;
        chips[missing].present=false;
        ASSERT_TRUE(baroInit()); run(30);
        EXPECT_TRUE(baroIsHealthy()); EXPECT_EQ(multiBaroFusion.contributors,1);
        EXPECT_EQ(baro.baroPressure,chips[1-missing].pressure);
    }
}
TEST_F(MultiBaroTest, BothAbsentFailDetection) {
    chips[0].present=chips[1].present=false;
    EXPECT_FALSE(baroInit()); EXPECT_FALSE(sensors(SENSOR_BARO));
}
TEST_F(MultiBaroTest, NackIsExcludedAndRecoveryIsGradual) {
    ASSERT_TRUE(baroInit()); run(50);
    chips[0].ack=false; run(30);
    EXPECT_TRUE(baroIsHealthy()); EXPECT_EQ(multiBaroFusion.contributors,1);
    EXPECT_EQ(baro.baroPressure,100020);
    EXPECT_FLOAT_EQ(multiBaroFusion.member[0].confidence,0);
    chips[0].ack=true; run(10);
    EXPECT_GT(multiBaroFusion.member[0].confidence,0);
    EXPECT_LT(multiBaroFusion.member[0].confidence,0.2f);
}
TEST_F(MultiBaroTest, AllReadFailuresStopNewNavigationSamplesAndHealth) {
    ASSERT_TRUE(baroInit()); run(50);
    chips[0].ack=chips[1].ack=false; run(30);
    int32_t previous=baro.baroPressure;
    run(30);
    EXPECT_FALSE(baroIsHealthy()); EXPECT_FALSE(baroHasNewSample());
    EXPECT_EQ(baro.baroPressure,previous);
}
TEST_F(MultiBaroTest, NoNewPressureMeansNoNewNavigationSample) {
    ASSERT_TRUE(baroInit());
    EXPECT_FALSE(baroHasNewSample());
    baroUpdate(); EXPECT_FALSE(baroHasNewSample());
}
TEST_F(MultiBaroTest, PressureSpikeLosesWeightWhileOtherSensorContinues) {
    ASSERT_TRUE(baroInit()); run(50);
    chips[0].pressure=110000; run(20);
    EXPECT_LT(multiBaroFusion.member[0].confidence,0.001f);
    EXPECT_LT(baro.baroPressure,100030);
    EXPECT_TRUE(baroIsHealthy());
}

TEST_F(MultiBaroTest, ModeChangeIsLatchedUntilReboot) {
    ASSERT_TRUE(baroInit()); multiSensorConfigMutable()->baroEnabled=false;
    EXPECT_TRUE(multiSensorBaroEnabled()); run(50);
    EXPECT_EQ(multiBaroFusion.contributors,2);
}
TEST_F(MultiBaroTest, NewConfigurationDefaultsPreserveExistingSingleSensorModes) {
    EXPECT_FALSE(pgResetTemplate_multiSensorConfig.imuEnabled);
    EXPECT_FALSE(pgResetTemplate_multiSensorConfig.baroEnabled);
    EXPECT_EQ(pgResetTemplate_multiSensorConfig.accCalibratedMask,0);
    for (unsigned tag=0;tag<3;tag++) for (unsigned axis=0;axis<3;axis++) {
        EXPECT_EQ(pgResetTemplate_multiSensorConfig.accZero[tag][axis],0);
        EXPECT_EQ(pgResetTemplate_multiSensorConfig.accGain[tag][axis],4096);
    }
    EXPECT_EQ(BARO_MS5611,3); EXPECT_EQ(BARO_SPL06,7);
}

TEST_F(MultiBaroTest, ParameterGroupStoresAndRestoresModesThresholdsAndPerTagCalibration) {
    uint8_t stored[sizeof(multiSensorConfig_t)];
    multiSensorConfigMutable()->imuEnabled=true;
    multiSensorConfigMutable()->recoveryMs=1234;
    multiSensorConfigMutable()->accCalibratedMask=5;
    multiSensorConfigMutable()->accZero[2][1]=-123;
    multiSensorConfigMutable()->accGain[2][1]=4500;
    ASSERT_EQ(pgStore(&multiSensorConfig_Registry,stored,sizeof(stored),0),sizeof(stored));
    pgReset(&multiSensorConfig_Registry,0);
    EXPECT_FALSE(multiSensorConfig()->imuEnabled);
    EXPECT_FALSE(multiSensorConfig()->baroEnabled);
    EXPECT_EQ(multiSensorConfig()->accCalibratedMask,0);
    pgLoad(&multiSensorConfig_Registry,0,stored,sizeof(stored),0);
    EXPECT_TRUE(multiSensorConfig()->imuEnabled);
    EXPECT_TRUE(multiSensorConfig()->baroEnabled);
    EXPECT_EQ(multiSensorConfig()->recoveryMs,1234);
    EXPECT_EQ(multiSensorConfig()->accCalibratedMask,5);
    EXPECT_EQ(multiSensorConfig()->accZero[2][1],-123);
    EXPECT_EQ(multiSensorConfig()->accGain[2][1],4500);
}
TEST_F(MultiBaroTest, IncompatibleParameterGroupVersionKeepsDisabledDefaults) {
    uint8_t stored[sizeof(multiSensorConfig_t)];
    memset(stored,0xff,sizeof(stored));
    pgLoad(&multiSensorConfig_Registry,0,stored,sizeof(stored),1);
    EXPECT_FALSE(multiSensorConfig()->imuEnabled);
    EXPECT_FALSE(multiSensorConfig()->baroEnabled);
    EXPECT_EQ(multiSensorConfig()->accGain[2][1],4096);
    EXPECT_EQ(multiSensorConfig()->accCalibratedMask,0);
}

TEST_F(MultiBaroTest, ARejectedNewSampleCannotFreshenAnOldHealthyNavigationSample) {
    ASSERT_TRUE(baroInit()); run(50);
    chips[1].pressure=110000;
    unsigned observed=0;
    for (unsigned tick=0;tick<100;tick++) {
        unsigned first=chips[0].pressureReads,second=chips[1].pressureReads;
        uint32_t wait=baroUpdate();
        if (chips[0].pressureReads==first && chips[1].pressureReads>second) {
            EXPECT_FALSE(baroHasNewSample()); observed++;
        }
        clockUs+=wait;
    }
    EXPECT_GT(observed,0u);
}
TEST_F(MultiBaroTest, StartupOffsetsDoNotCreateAltitudeStepWhenEitherBarometerFails) {
    chips[1].pressure=100200;
    ASSERT_TRUE(baroInit());
    barometerConfigMutable()->baro_calibration_tolerance=150;
    baroStartCalibration();
    for (unsigned tick=0;tick<1000;tick++) {
        uint32_t wait=baroUpdate();
        if (baroHasNewSample()) baroCalculateAltitude();
        clockUs+=wait;
    }
    ASSERT_TRUE(baroIsCalibrationComplete());
    EXPECT_NEAR(baroGetPressureOffset(0),100,0.01);
    EXPECT_NEAR(baroGetPressureOffset(1),-100,0.01);
    EXPECT_NEAR(baroGetLatestAltitude(),0,1);
    for (unsigned lost=0;lost<2;lost++) {
        chips[lost].ack=false;
        for (unsigned tick=0;tick<50;tick++) {
            uint32_t wait=baroUpdate();
            if (baroHasNewSample()) baroCalculateAltitude();
            clockUs+=wait;
        }
        EXPECT_TRUE(baroIsHealthy()); EXPECT_NEAR(baroGetLatestAltitude(),0,1);
        chips[lost].ack=true; run(1000);
    }
}
TEST_F(MultiBaroTest, PresentButUnreadableBarometerCannotFinishStartupPairCalibration) {
    chips[1].ack=false;
    ASSERT_TRUE(baroInit()); baroStartCalibration();
    for (unsigned tick=0;tick<1000;tick++) {
        uint32_t wait=baroUpdate();
        if (baroHasNewSample()) baroCalculateAltitude();
        clockUs+=wait;
    }
    EXPECT_FALSE(baroIsCalibrationComplete());
}

TEST_F(MultiBaroTest, GrossStartupDisagreementCannotBeHiddenByOffsetCalibration) {
    chips[1].pressure=110000;
    ASSERT_TRUE(baroInit()); baroStartCalibration();
    for (unsigned tick=0;tick<1000;tick++) {
        uint32_t wait=baroUpdate();
        if (baroHasNewSample()) baroCalculateAltitude();
        clockUs+=wait;
    }
    EXPECT_FALSE(baroIsCalibrationComplete());
    EXPECT_FLOAT_EQ(baroGetPressureOffset(0),0);
    EXPECT_FLOAT_EQ(baroGetPressureOffset(1),0);
}

TEST_F(MultiBaroTest, PhysicallyAbsentSecondBarometerDoesNotBlockGroundCalibration) {
    chips[0].present=false;
    ASSERT_TRUE(baroInit()); baroStartCalibration();
    for (unsigned tick=0;tick<1000;tick++) {
        uint32_t wait=baroUpdate();
        if (baroHasNewSample()) baroCalculateAltitude();
        clockUs+=wait;
    }
    EXPECT_TRUE(baroIsCalibrationComplete()); EXPECT_TRUE(baroIsHealthy());
    EXPECT_EQ(multiBaroFusion.contributors,1);
    EXPECT_FLOAT_EQ(baroGetPressureOffset(1),0);
    EXPECT_NEAR(baroGetLatestAltitude(),0,1);
}

TEST_F(MultiBaroTest, IndividualTelemetryHeightsShareGroundDatumAndUpdateOnConversion) {
    chips[1].pressure = 100200;
    ASSERT_TRUE(baroInit());
    barometerConfigMutable()->baro_calibration_tolerance = 150;
    baroStartCalibration();
    int32_t first = 123, second = 456;
    EXPECT_FALSE(baroGetSensorAltitude(0, &first));
    EXPECT_FALSE(baroGetSensorAltitude(1, &second));
    for (unsigned tick = 0; tick < 1000; tick++) {
        const uint32_t wait = baroUpdate();
        if (baroHasNewSample()) baroCalculateAltitude();
        clockUs += wait;
    }
    ASSERT_TRUE(baroIsCalibrationComplete());
    ASSERT_TRUE(baroGetSensorAltitude(0, &first));
    ASSERT_TRUE(baroGetSensorAltitude(1, &second));
    EXPECT_NEAR(first, 0, 2); EXPECT_NEAR(second, 0, 2);
    EXPECT_EQ(baroGetSensorTemperature(0), 2000);
    EXPECT_EQ(baroGetSensorTemperature(1), 2000);
    EXPECT_FALSE(baroGetSensorAltitude(2, &first));
    EXPECT_FALSE(baroGetSensorAltitude(0, nullptr));
    chips[0].pressure += 50;
    run(30);
    ASSERT_TRUE(baroGetSensorAltitude(0, &first));
    ASSERT_TRUE(baroGetSensorAltitude(1, &second));
    EXPECT_LT(first, -300); EXPECT_NEAR(second, 0, 2);
    baroStartCalibration();
    EXPECT_FALSE(baroGetSensorAltitude(0, &first));
    EXPECT_FALSE(baroGetSensorAltitude(1, &second));
}

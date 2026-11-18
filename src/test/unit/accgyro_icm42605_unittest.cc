/*
 * This file is part of INAV.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <cstring>
#include <cstdint>

extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "common/log.h"
#include "drivers/time.h"
#include "drivers/bus.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm42605.h"
}
#include "gtest/gtest.h"

namespace {
busDevice_t devices[2];
uint8_t registers[2][5][256];
uint8_t banks[2];
uint32_t scratch[2][(BUS_SCRATCHPAD_MEMORY_SIZE + 3) / 4];
bool available[2];
int failedReadRegister;
int failedWriteRegister;
bool poweredWithInterrupts;
unsigned deinitCount;
unsigned indexOf(const busDevice_t *dev) { return dev - devices; }
}

extern "C" {
busDevice_t *busDeviceInit(busType_e bus, devHardwareType_e hw, uint8_t tag, resourceOwner_e owner)
{
    UNUSED(bus); UNUSED(owner);
    EXPECT_EQ(hw, DEVHW_ICM42605);
    return tag < 2 && available[tag] ? &devices[tag] : nullptr;
}
busDevice_t *busDeviceOpen(busType_e bus, devHardwareType_e hw, uint8_t tag)
{
    return busDeviceInit(bus, hw, tag, OWNER_MPU);
}
void busDeviceDeInit(busDevice_t *dev) { UNUSED(dev); deinitCount++; }
void *busDeviceGetScratchpadMemory(const busDevice_t *dev) { return scratch[indexOf(dev)]; }
void busSetSpeed(const busDevice_t *dev, busSpeed_e speed) { UNUSED(dev); UNUSED(speed); }
bool busWrite(const busDevice_t *dev, uint8_t reg, uint8_t value)
{
    const unsigned index = indexOf(dev);
    if (reg == failedWriteRegister) { return false; }
    if (reg == 0x4e && value == 0x0f && registers[index][0][0x65] != 0) { poweredWithInterrupts = true; }
    if (reg == 0x76) {
        banks[index] = value;
    } else {
        registers[index][banks[index]][reg] = value;
    }
    return true;
}
bool busReadBuf(const busDevice_t *dev, uint8_t reg, uint8_t *values, uint8_t length)
{
    if (reg == failedReadRegister) { return false; }
    const unsigned index = indexOf(dev);
    memcpy(values, registers[index][banks[index]] + reg, length);
    return true;
}
bool busRead(const busDevice_t *dev, uint8_t reg, uint8_t *value)
{
    return busReadBuf(dev, reg, value, 1);
}
void delay(timeMs_t ms) { UNUSED(ms); }
void _logf(logTopic_e topic, unsigned level, const char *fmt, ...)
{
    UNUSED(topic); UNUSED(level); UNUSED(fmt);
}
}

class Icm426xxBusTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(devices, 0, sizeof(devices));
        memset(registers, 0, sizeof(registers));
        memset(scratch, 0, sizeof(scratch));
        memset(banks, 0, sizeof(banks));
        memset(gyros, 0, sizeof(gyros));
        memset(accels, 0, sizeof(accels));
        available[0] = available[1] = true;
        failedReadRegister = failedWriteRegister = -1;
        poweredWithInterrupts = false;
        deinitCount = 0;
        for (unsigned i = 0; i < 2; i++) {
            gyros[i].imuSensorToUse = accels[i].imuSensorToUse = i;
            gyros[i].requestedSampleIntervalUs = 1000;
            gyros[i].lpf = GYRO_LPF_256HZ;
            devices[i].param = i ? CW90_DEG : CW0_DEG;
        }
    }
    gyroDev_t gyros[2];
    accDev_t accels[2];
};

TEST_F(Icm426xxBusTest, ExistingIdentitiesKeepFullScaleOdrAndAafSettings)
{
    const uint8_t ids[] = {ICM42605_WHO_AM_I_CONST, ICM42686P_WHO_AM_I_CONST, ICM42688P_WHO_AM_I_CONST};
    for (unsigned i = 0; i < ARRAYLEN(ids); i++) {
        registers[0][0][MPU_RA_WHO_AM_I] = ids[i];
        ASSERT_TRUE(icm42605GyroDetect(&gyros[0]));
        gyros[0].initFn(&gyros[0]);
        EXPECT_EQ(gyros[0].sampleRateIntervalUs, 1000);
        EXPECT_FLOAT_EQ(gyros[0].scale, 1.0f / 16.4f);
        EXPECT_EQ(registers[0][0][0x4e], 0x0f);
        EXPECT_EQ(registers[0][0][0x4f], i == 1 ? 0x26 : 0x06);
        EXPECT_EQ(registers[0][0][0x50], i == 1 ? 0x26 : 0x06);
        EXPECT_EQ(registers[0][0][0x52], 0xff);
        EXPECT_EQ(registers[0][0][0x65], 0x08);
        EXPECT_EQ(registers[0][1][0x0c], i == 0 ? 21 : 6);
        EXPECT_EQ(registers[0][2][0x03], i == 0 ? 42 : 12);
        EXPECT_EQ(banks[0], 0);
    }
}

TEST_F(Icm426xxBusTest, ExistingSampleRatesRemainSelectable)
{
    registers[0][0][MPU_RA_WHO_AM_I] = ICM42688P_WHO_AM_I_CONST;
    ASSERT_TRUE(icm42605GyroDetect(&gyros[0]));
    const uint32_t intervals[] = {125, 250, 500, 1000, 2000};
    const uint8_t odrs[] = {3, 4, 5, 6, 15};
    for (unsigned i = 0; i < ARRAYLEN(intervals); i++) {
        gyros[0].requestedSampleIntervalUs = intervals[i];
        gyros[0].initFn(&gyros[0]);
        EXPECT_EQ(gyros[0].sampleRateIntervalUs, intervals[i]);
        EXPECT_EQ(registers[0][0][0x4f], odrs[i]);
        EXPECT_EQ(registers[0][0][0x50], odrs[i]);
    }
}

TEST_F(Icm426xxBusTest, TwoIdenticalDevicesKeepIndependentReadsAndAlignment)
{
    const uint8_t gyroValues[] = {0x7f, 0xff, 0x80, 0, 0xff, 0xfe};
    const uint8_t accValues[] = {0, 8, 0xff, 0xfc, 0x12, 0x34};
    for (unsigned i = 0; i < 2; i++) {
        registers[i][0][MPU_RA_WHO_AM_I] = ICM42688P_WHO_AM_I_CONST;
        ASSERT_TRUE(icm42605GyroDetect(&gyros[i]));
        gyros[i].initFn(&gyros[i]);
        ASSERT_TRUE(icm42605AccDetect(&accels[i]));
        accels[i].initFn(&accels[i]);
        EXPECT_EQ(gyros[i].gyroAlign, devices[i].param);
        EXPECT_EQ(accels[i].accAlign, devices[i].param);
        EXPECT_EQ(accels[i].acc_1G, 2048);
        memcpy(registers[i][0] + 0x25, gyroValues, sizeof(gyroValues));
        memcpy(registers[i][0] + 0x1f, accValues, sizeof(accValues));
    }
    registers[1][0][0x26] = 0xf0;
    ASSERT_TRUE(gyros[0].readFn(&gyros[0]));
    ASSERT_TRUE(gyros[1].readFn(&gyros[1]));
    EXPECT_EQ(gyros[0].gyroADCRaw[0], 32767);
    EXPECT_EQ(gyros[1].gyroADCRaw[0], 32752);
    EXPECT_EQ(gyros[0].gyroADCRaw[1], -32768);
    EXPECT_EQ(gyros[0].gyroADCRaw[2], -2);
    ASSERT_TRUE(accels[0].readFn(&accels[0]));
    EXPECT_EQ(accels[0].ADCRaw[0], 8);
    EXPECT_EQ(accels[0].ADCRaw[1], -4);
    EXPECT_EQ(accels[0].ADCRaw[2], 4660);
}

TEST_F(Icm426xxBusTest, FailedDataReadsPreserveOutputsAndNextReadRecovers)
{
    registers[0][0][MPU_RA_WHO_AM_I] = ICM42605_WHO_AM_I_CONST;
    ASSERT_TRUE(icm42605GyroDetect(&gyros[0]));
    ASSERT_TRUE(icm42605AccDetect(&accels[0]));
    gyros[0].gyroADCRaw[0] = 123;
    accels[0].ADCRaw[0] = 456;
    failedReadRegister = 0x25;
    EXPECT_FALSE(gyros[0].readFn(&gyros[0]));
    EXPECT_EQ(gyros[0].gyroADCRaw[0], 123);
    failedReadRegister = 0x1f;
    EXPECT_FALSE(accels[0].readFn(&accels[0]));
    EXPECT_EQ(accels[0].ADCRaw[0], 456);
    int16_t temperature = 99;
    failedReadRegister = 0x1d;
    EXPECT_FALSE(gyros[0].temperatureFn(&gyros[0], &temperature));
    EXPECT_EQ(temperature, 99);
    failedReadRegister = -1;
    EXPECT_TRUE(gyros[0].readFn(&gyros[0]));
    EXPECT_TRUE(accels[0].readFn(&accels[0]));
    EXPECT_TRUE(gyros[0].temperatureFn(&gyros[0], &temperature));
    EXPECT_EQ(temperature, 250);
}

TEST_F(Icm426xxBusTest, MissingBusWrongIdentityAndUnpairedAccelAreRejected)
{
    available[0] = false;
    EXPECT_FALSE(icm42605GyroDetect(&gyros[0]));
    available[0] = true;
    EXPECT_FALSE(icm42605AccDetect(&accels[0]));
    EXPECT_FALSE(icm42605GyroDetect(&gyros[0]));
    EXPECT_EQ(deinitCount, 1u);
}

TEST_F(Icm426xxBusTest, PollingModeDisablesSharedInterruptBeforePoweringUp) {
    registers[0][0][MPU_RA_WHO_AM_I] = ICM42688P_WHO_AM_I_CONST;
    registers[0][0][0x65] = 0x08;
    ASSERT_TRUE(icm42605GyroDetect(&gyros[0]));
    gyros[0].pollingOnly = true;
    gyros[0].initFn(&gyros[0]);
    EXPECT_EQ(registers[0][0][0x65],0);
    EXPECT_FALSE(poweredWithInterrupts);
    EXPECT_NE(gyros[0].sampleRateIntervalUs,0u);
}
TEST_F(Icm426xxBusTest, PollingInitializationFailureDoesNotWakeSensor) {
    registers[0][0][MPU_RA_WHO_AM_I] = ICM42688P_WHO_AM_I_CONST;
    ASSERT_TRUE(icm42605GyroDetect(&gyros[0]));
    gyros[0].pollingOnly = true;
    failedWriteRegister = 0x65;
    gyros[0].initFn(&gyros[0]);
    EXPECT_EQ(gyros[0].sampleRateIntervalUs,0u);
    EXPECT_EQ(registers[0][0][0x4e],0);
}
TEST_F(Icm426xxBusTest, IdentityNackCannotUseAStaleOrUninitializedIdentityByte) {
    registers[0][0][MPU_RA_WHO_AM_I] = ICM42688P_WHO_AM_I_CONST;
    failedReadRegister = MPU_RA_WHO_AM_I;
    EXPECT_FALSE(icm42605GyroDetect(&gyros[0]));
    EXPECT_EQ(deinitCount,1u);
}

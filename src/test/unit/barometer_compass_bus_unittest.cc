/*
 * This file is part of INAV.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include <cstdint>

extern "C" {
#include "platform.h"
#include "common/utils.h"
#include "common/axis.h"
#include "drivers/time.h"
#include "drivers/bus.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp388.h"
#include "drivers/compass/compass_lis3mdl.h"
}

#include "gtest/gtest.h"

namespace {
uint8_t registers[256];
busDevice_t device;
bool available;
int failedReadRegister;
int failedWriteRegister;
unsigned deinitCount;
unsigned identityReads;
uint8_t requestedTag;
devHardwareType_e requestedHardware;
}

extern "C" {
busDevice_t *busDeviceInit(busType_e bus, devHardwareType_e hardware, uint8_t tag, resourceOwner_e owner)
{
    UNUSED(owner);
    requestedTag = tag;
    requestedHardware = hardware;
    if (bus != BUSTYPE_ANY) {
        EXPECT_EQ(bus, device.busType);
    }
    return available ? &device : nullptr;
}

void busDeviceDeInit(busDevice_t *dev)
{
    EXPECT_EQ(dev, &device);
    deinitCount++;
}

void busSetSpeed(const busDevice_t *dev, busSpeed_e speed)
{
    EXPECT_EQ(dev, &device);
    EXPECT_EQ(speed, BUS_SPEED_STANDARD);
}

bool busWrite(const busDevice_t *dev, uint8_t reg, uint8_t value)
{
    EXPECT_EQ(dev, &device);
    if (reg == failedWriteRegister) {
        return false;
    }
    registers[reg] = value;
    return true;
}

bool busReadBuf(const busDevice_t *dev, uint8_t reg, uint8_t *values, uint8_t length)
{
    EXPECT_EQ(dev, &device);
    if (reg == 0 || reg == 0x0f) {
        identityReads++;
    }
    if (reg == failedReadRegister) {
        return false; // A failed transaction deliberately leaves the destination untouched.
    }
    if (device.busType == BUSTYPE_SPI) {
        // BMP3 returns one dummy byte before its register data on SPI.
        *values++ = 0xff;
        length--;
    }
    memcpy(values, registers + reg, length);
    return true;
}

bool busRead(const busDevice_t *dev, uint8_t reg, uint8_t *value)
{
    return busReadBuf(dev, reg, value, 1);
}

void delay(timeMs_t ms)
{
    UNUSED(ms);
}
}

class ExistingSensorBusTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(registers, 0, sizeof(registers));
        memset(&device, 0, sizeof(device));
        memset(&baro, 0, sizeof(baro));
        memset(&mag, 0, sizeof(mag));
        device.busType = BUSTYPE_I2C;
        available = true;
        failedReadRegister = -1;
        failedWriteRegister = -1;
        deinitCount = identityReads = 0;
    }

    void prepareBmp388()
    {
        registers[0] = 0x50;
        // Synthetic, exactly soluble trim vector: zero temperature, constant 50000 Pa.
        // P5 = 16000 -> 3.125 * P5 Pa; P1 = 16384 removes pressure sensitivity.
        registers[0x36] = 0;
        registers[0x37] = 0x40;
        registers[0x3c] = 0x80;
        registers[0x3d] = 0x3e;
    }

    void expectBmpMeasurement()
    {
        ASSERT_TRUE(baro.get_up(&baro));
        int32_t pressure = -1;
        int32_t temperature = -1;
        ASSERT_TRUE(baro.calculate(&baro, &pressure, &temperature));
        EXPECT_EQ(pressure, 50000);
        EXPECT_EQ(temperature, 0);
    }

    baroDev_t baro;
    magDev_t mag;
};

TEST_F(ExistingSensorBusTest, MissingBusDoesNotDetectEitherSensor)
{
    available = false;
    EXPECT_FALSE(bmp388Detect(&baro));
    EXPECT_FALSE(lis3mdlDetect(&mag));
    EXPECT_EQ(deinitCount, 0u);
}

TEST_F(ExistingSensorBusTest, Bmp388RejectsWrongIdentityAndReleasesBus)
{
    EXPECT_FALSE(bmp388Detect(&baro));
    EXPECT_EQ(identityReads, 5u);
    EXPECT_EQ(deinitCount, 1u);
}

TEST_F(ExistingSensorBusTest, Bmp388RejectsIdentityReadFailure)
{
    prepareBmp388();
    failedReadRegister = 0;
    EXPECT_FALSE(bmp388Detect(&baro));
    EXPECT_EQ(deinitCount, 1u);
}

TEST_F(ExistingSensorBusTest, Bmp388I2cInitializesForcedModeAndConvertsToPaAndCentiC)
{
    prepareBmp388();
    ASSERT_TRUE(bmp388Detect(&baro));
    EXPECT_EQ(requestedHardware, DEVHW_BMP388);
    EXPECT_EQ(registers[0x1b], 0x13);
    EXPECT_EQ(registers[0x1c], 0x03);
    EXPECT_EQ(baro.ut_delay, 0);
    EXPECT_EQ(baro.up_delay, 36939);
    ASSERT_TRUE(baro.start_ut(&baro));
    ASSERT_TRUE(baro.get_ut(&baro));
    ASSERT_TRUE(baro.start_up(&baro));
    expectBmpMeasurement();
}

TEST_F(ExistingSensorBusTest, Bmp388SpiSkipsDummyByteForIdCalibrationAndMeasurements)
{
    prepareBmp388();
    device.busType = BUSTYPE_SPI;
    ASSERT_TRUE(bmp388Detect(&baro));
    expectBmpMeasurement();
}

TEST_F(ExistingSensorBusTest, Bmp390RetainsSharedDriverSupport)
{
    prepareBmp388();
    registers[0] = 0x60;
    ASSERT_TRUE(bmp388Detect(&baro));
    expectBmpMeasurement();
}

TEST_F(ExistingSensorBusTest, Bmp388PropagatesMeasurementStartFailure)
{
    prepareBmp388();
    ASSERT_TRUE(bmp388Detect(&baro));
    failedWriteRegister = 0x1b;
    EXPECT_FALSE(baro.start_up(&baro));
    failedWriteRegister = -1;
    EXPECT_TRUE(baro.start_up(&baro));
}

TEST_F(ExistingSensorBusTest, Lis3mdlInitializesExpectedRegistersAndUsesSelectedTag)
{
    registers[0x0f] = 0x3d;
    mag.magSensorToUse = 1;
    ASSERT_TRUE(lis3mdlDetect(&mag));
    EXPECT_EQ(requestedHardware, DEVHW_LIS3MDL);
    EXPECT_EQ(requestedTag, 1);
    ASSERT_TRUE(mag.init(&mag));
    EXPECT_EQ(registers[0x20], 0xfc);
    EXPECT_EQ(registers[0x21], 0);
    EXPECT_EQ(registers[0x22], 0);
    EXPECT_EQ(registers[0x23], 0x0c);
    EXPECT_EQ(registers[0x24], 0x40);
}

TEST_F(ExistingSensorBusTest, Lis3mdlRejectsWrongIdentityAndReadFailure)
{
    EXPECT_FALSE(lis3mdlDetect(&mag));
    EXPECT_EQ(identityReads, 5u);
    registers[0x0f] = 0x3d;
    failedReadRegister = 0x0f;
    EXPECT_FALSE(lis3mdlDetect(&mag));
    EXPECT_EQ(identityReads, 10u);
    EXPECT_EQ(deinitCount, 2u);
}

TEST_F(ExistingSensorBusTest, Lis3mdlDecodesSignedLittleEndianAxesWithoutReordering)
{
    registers[0x0f] = 0x3d;
    ASSERT_TRUE(lis3mdlDetect(&mag));
    registers[0x27] = 0x08;
    const uint8_t values[] = {0xfc, 0x7f, 0, 0x80, 0x34, 0x12};
    memcpy(registers + 0x28, values, sizeof(values));
    ASSERT_TRUE(mag.read(&mag));
    EXPECT_EQ(mag.magADCRaw[0], 8191);
    EXPECT_EQ(mag.magADCRaw[1], -8192);
    EXPECT_EQ(mag.magADCRaw[2], 1165);
}

TEST_F(ExistingSensorBusTest, Lis3mdlWaitsForReadyAndRecoversAfterTransportFailure)
{
    registers[0x0f] = 0x3d;
    ASSERT_TRUE(lis3mdlDetect(&mag));
    mag.magADCRaw[0] = 123;
    EXPECT_FALSE(mag.read(&mag));
    EXPECT_EQ(mag.magADCRaw[0], 123);
    registers[0x27] = 0x08;
    failedReadRegister = 0x27;
    EXPECT_FALSE(mag.read(&mag));
    EXPECT_EQ(mag.magADCRaw[0], 123);
    failedReadRegister = 0x28;
    EXPECT_FALSE(mag.read(&mag));
    // Preserve the existing driver's documented failure output, including zeroing.
    EXPECT_EQ(mag.magADCRaw[0], 0);
    failedReadRegister = -1;
    registers[0x28] = 40;
    ASSERT_TRUE(mag.read(&mag));
    EXPECT_EQ(mag.magADCRaw[0], 10);
}

// Baseline characterization, not a claim that this error handling is sufficient.
// These tests make existing shortcomings visible before adding a new barometer.
TEST_F(ExistingSensorBusTest, Bmp388ReadNackCurrentlyReportsSuccessWithStaleSample)
{
    prepareBmp388();
    ASSERT_TRUE(bmp388Detect(&baro));
    expectBmpMeasurement();
    failedReadRegister = 0x04;
    EXPECT_TRUE(baro.get_up(&baro));
    int32_t pressure = -1;
    EXPECT_TRUE(baro.calculate(&baro, &pressure, nullptr));
    EXPECT_EQ(pressure, 50000);
    failedReadRegister = -1;
    expectBmpMeasurement();
}

TEST_F(ExistingSensorBusTest, Bmp388CalibrationNackCurrentlyDoesNotFailDetection)
{
    prepareBmp388();
    failedReadRegister = 0x31;
    EXPECT_TRUE(bmp388Detect(&baro));
}

TEST_F(ExistingSensorBusTest, Lis3mdlConfigurationNackCurrentlyDoesNotFailInitialization)
{
    registers[0x0f] = 0x3d;
    ASSERT_TRUE(lis3mdlDetect(&mag));
    failedWriteRegister = 0x20;
    EXPECT_TRUE(mag.init(&mag));
    EXPECT_EQ(registers[0x20], 0);
}

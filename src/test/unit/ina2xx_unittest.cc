/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify this software
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include "gtest/gtest.h"

extern "C" {
    #include <stdbool.h>
    #include <stdint.h>
    #include <string.h>

    #include "drivers/ina2xx.h"
}

#define INA2XX_REG_VBUS             0x05
#define INA2XX_REG_CURRENT          0x07
#define INA2XX_REG_CONFIG           0x00
#define INA2XX_REG_SHUNT_CAL        0x02
#define INA2XX_REG_MANUFACTURER_ID  0x3E
#define INA2XX_REG_DEVICE_ID        0x3F

static busDevice_t fakeBusDevice;
static uint32_t fakeRegisters[256];
static uint8_t fakeRegisterLength[256];
static bool fakeRegisterReadable[256];
static bool fakeInitSucceeds;
static bool fakeDeInitCalled;
static uint8_t fakeInitTag;

static void resetFakeBus(void)
{
    memset(&fakeBusDevice, 0, sizeof(fakeBusDevice));
    memset(fakeRegisters, 0, sizeof(fakeRegisters));
    memset(fakeRegisterLength, 0, sizeof(fakeRegisterLength));
    memset(fakeRegisterReadable, 0, sizeof(fakeRegisterReadable));
    fakeInitSucceeds = true;
    fakeDeInitCalled = false;
    fakeInitTag = 0;
    fakeBusDevice.busType = BUSTYPE_I2C;
}

static void setFakeRegister(uint8_t reg, uint32_t value, uint8_t length)
{
    fakeRegisters[reg] = value;
    fakeRegisterLength[reg] = length;
    fakeRegisterReadable[reg] = true;
}

extern "C" {
    busDevice_t *busDeviceInit(busType_e bus, devHardwareType_e hw, uint8_t tag, resourceOwner_e owner)
    {
        EXPECT_EQ(BUSTYPE_I2C, bus);
        EXPECT_EQ(DEVHW_INA2XX, hw);
        EXPECT_EQ(OWNER_CURRENT_METER, owner);
        fakeInitTag = tag;
        return fakeInitSucceeds ? &fakeBusDevice : nullptr;
    }

    void busDeviceDeInit(busDevice_t *dev)
    {
        EXPECT_EQ(&fakeBusDevice, dev);
        fakeDeInitCalled = true;
    }

    bool busReadBuf(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
    {
        EXPECT_EQ(&fakeBusDevice, busdev);
        if (length != fakeRegisterLength[reg] || !fakeRegisterReadable[reg]) {
            return false;
        }

        for (uint8_t i = 0; i < length; i++) {
            const uint8_t shift = (length - i - 1) * 8;
            data[i] = (fakeRegisters[reg] >> shift) & 0xFF;
        }
        return true;
    }

    bool busWriteBuf(const busDevice_t *busdev, uint8_t reg, const uint8_t *data, uint8_t length)
    {
        EXPECT_EQ(&fakeBusDevice, busdev);
        EXPECT_EQ(2, length);
        if (length != 2) {
            return false;
        }

        fakeRegisters[reg] = ((uint32_t)data[0] << 8) | data[1];
        fakeRegisterLength[reg] = length;
        fakeRegisterReadable[reg] = true;
        return true;
    }
}

TEST(INA2XX, DetectsINA228AndConvertsMeasurements)
{
    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x5449, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2281, 2);
    setFakeRegister(INA2XX_REG_VBUS, 245760U << 4, 3); // 48 V
    setFakeRegister(INA2XX_REG_CURRENT, 25600U << 4, 3); // 10 A at 204.8 A full scale

    ina2xxDev_t dev = {};
    ASSERT_TRUE(ina2xxInit(&dev, I2CDEV_2, 0x45, 0));
    EXPECT_EQ(INA2XX_DEVICE_228, dev.type);
    EXPECT_EQ(I2CDEV_2, dev.busDev->busdev.i2c.i2cBus);
    EXPECT_EQ(0x45, dev.busDev->busdev.i2c.address);
    EXPECT_EQ(0, fakeInitTag);
    EXPECT_EQ(0, fakeRegisters[INA2XX_REG_CONFIG]);
    EXPECT_EQ(1024, fakeRegisters[INA2XX_REG_SHUNT_CAL]);

    uint16_t voltage = 0;
    int16_t current = 0;
    EXPECT_TRUE(ina2xxReadBusVoltage(&dev, &voltage));
    EXPECT_TRUE(ina2xxReadCurrent(&dev, &current));
    EXPECT_EQ(4800, voltage);
    EXPECT_EQ(1000, current);
}

TEST(INA2XX, DetectsINA238AndConvertsMeasurements)
{
    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x5449, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2381, 2);
    setFakeRegister(INA2XX_REG_VBUS, 15360, 2); // 48 V
    setFakeRegister(INA2XX_REG_CURRENT, 1600, 2); // 10 A at 204.8 A full scale

    ina2xxDev_t dev = {};
    ASSERT_TRUE(ina2xxInit(&dev, I2CDEV_2, 0x44, 1));
    EXPECT_EQ(INA2XX_DEVICE_238, dev.type);
    EXPECT_EQ(1, fakeInitTag);

    uint16_t voltage = 0;
    int16_t current = 0;
    EXPECT_TRUE(ina2xxReadBusVoltage(&dev, &voltage));
    EXPECT_TRUE(ina2xxReadCurrent(&dev, &current));
    EXPECT_EQ(4800, voltage);
    EXPECT_EQ(1000, current);
}

TEST(INA2XX, RejectsUnexpectedChipIdAndI2cErrors)
{
    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x1234, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2281, 2);

    ina2xxDev_t dev = {};
    EXPECT_FALSE(ina2xxInit(&dev, I2CDEV_2, 0x41, 2));
    EXPECT_EQ(nullptr, dev.busDev);
    EXPECT_TRUE(fakeDeInitCalled);

    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x5449, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2281, 2);
    ASSERT_TRUE(ina2xxInit(&dev, I2CDEV_2, 0x41, 2));
    uint16_t voltage = 1;
    int16_t current = 1;
    EXPECT_FALSE(ina2xxReadBusVoltage(&dev, &voltage));
    EXPECT_FALSE(ina2xxReadCurrent(&dev, &current));
}

TEST(INA2XX, HandlesNegativeCurrent)
{
    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x5449, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2381, 2);
    setFakeRegister(INA2XX_REG_CURRENT, static_cast<uint16_t>(-1600), 2);

    ina2xxDev_t dev = {};
    ASSERT_TRUE(ina2xxInit(&dev, I2CDEV_1, 0x45, 0));
    int16_t current = 0;
    EXPECT_TRUE(ina2xxReadCurrent(&dev, &current));
    EXPECT_EQ(-1000, current);

    resetFakeBus();
    setFakeRegister(INA2XX_REG_MANUFACTURER_ID, 0x5449, 2);
    setFakeRegister(INA2XX_REG_DEVICE_ID, 0x2281, 2);
    setFakeRegister(INA2XX_REG_CURRENT, ((1U << 20) - 25600U) << 4, 3);

    ina2xxDev_t ina228 = {};
    ASSERT_TRUE(ina2xxInit(&ina228, I2CDEV_2, 0x45, 0));
    current = 0;
    EXPECT_TRUE(ina2xxReadCurrent(&ina228, &current));
    EXPECT_EQ(-1000, current);
}

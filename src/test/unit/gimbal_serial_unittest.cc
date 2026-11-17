/*
 * This file is part of INAV.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>

#include "gtest/gtest.h"
#include "unittest_macros.h"

#include "io/gimbal_serial.h"
#include "drivers/headtracker_common.h"

void dumpMemory(uint8_t *mem, int size)
{
    for(int i =0; i < size; ++i) {
        printf("%02x ", mem[i]);
    }
    printf("\n");
}

extern "C" {
timeUs_t micros(void) {
    return 10;
}
}

TEST(GimbalSerialTest, TestGimbalSerialScale)
{
    int16_t res16 = gimbal_scale12(1000, 2000, 2000);
    EXPECT_TRUE(res16 == 2047);
    res16 = gimbal_scale12(1000, 2000, 1000);
    printf("res16: %i\n", res16);
    EXPECT_TRUE(res16 == -2048);
    res16 = gimbal_scale12(1000, 2000, 1500);
    printf("res16: %i\n", res16);
    EXPECT_TRUE(res16 == -1);
    res16 = gimbal_scale12(1000, 2000, 1501);
    printf("res16: %i\n", res16);
    EXPECT_TRUE(res16 == 3);
}
#include "io/gimbal_mode_logic.h"
#include <cstring>
#include <cstddef>
extern "C" {
void pgResetFn_gimbalConfig(gimbalConfig_t *);
bool pgMigrateFn_gimbalConfig(void *, const void *, uint16_t, uint8_t);
}
TEST(GimbalMode, DisabledRetainsLegacyModes)
{
    gimbalModeSwitchState_t s = {};
    for (unsigned m = 0; m < 8; ++m)
        EXPECT_EQ(m, gimbalModeSelect(&s, 0, false, 0, false, false, m, 0));
}
TEST(GimbalMode, StableSwitchAndHysteresis)
{
    gimbalModeSwitchState_t s = {};
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, true, 1500, false, false, 3, 100));
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, true, 1500, false, false, 3, 249));
    EXPECT_EQ(1, gimbalModeSelect(&s, 8, true, 1500, false, false, 3, 250));
    EXPECT_EQ(1, gimbalModeSelect(&s, 8, true, 1290, false, false, 3, 500));
    EXPECT_EQ(1, gimbalModeSelect(&s, 8, true, 1310, false, false, 3, 700));
    EXPECT_EQ(1, gimbalModeSelect(&s, 8, true, 2000, false, false, 3, 800));
    EXPECT_EQ(2, gimbalModeSelect(&s, 8, true, 2000, false, false, 3, 950));
    EXPECT_EQ(2, gimbalModeSelect(&s, 8, true, 1690, false, false, 3, 1200));
    EXPECT_EQ(2, gimbalModeSelect(&s, 8, true, 1710, false, false, 3, 1400));
    EXPECT_EQ(2, gimbalModeSelect(&s, 8, true, 1000, false, false, 3, 1500));
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, true, 1000, false, false, 3, 1650));
}
TEST(GimbalMode, PrioritiesInvalidInputAndChannelChange)
{
    gimbalModeSwitchState_t s = {8, 2, 2, 0};
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, false, 2000, false, false, 3, 100));
    s = {8, 2, 2, 0};
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, true, 2000, true, false, 3, 100));
    s = {8, 2, 2, 0};
    EXPECT_EQ(0, gimbalModeSelect(&s, 8, true, 2000, false, true, 3, 100));
    s = {8, 2, 2, 0};
    EXPECT_EQ(0, gimbalModeSelect(&s, 9, true, 2000, false, false, 3, 100));
    EXPECT_EQ(0, gimbalModeSelect(&s, 0, true, 2000, true, true, 3, 100));
}
TEST(GimbalMode, TimerRollover)
{
    gimbalModeSwitchState_t s = {};
    EXPECT_EQ(0, gimbalModeSelect(&s, 1, true, 2000, false, false, 0, UINT32_MAX - 100));
    EXPECT_EQ(2, gimbalModeSelect(&s, 1, true, 2000, false, false, 0, 50));
}
TEST(GimbalConfig, MigrationPreservesLegacyFields)
{
    struct LegacyConfig {
        uint8_t panChannel, tiltChannel, rollChannel;
        int8_t sensitivity;
        int16_t panTrim, tiltTrim, rollTrim;
    } old = {8, 9, 10, -5, 100, -200, 300};
    ASSERT_EQ(sizeof(old), offsetof(gimbalConfig_t, modeChannel));
    gimbalConfig_t config;
    pgResetFn_gimbalConfig(&config);
    EXPECT_EQ(0, config.modeChannel);
    ASSERT_TRUE(pgMigrateFn_gimbalConfig(&config, &old, sizeof(old), 1));
    EXPECT_EQ(0, std::memcmp(&config, &old, sizeof(old)));
    EXPECT_EQ(0, config.modeChannel);
    pgResetFn_gimbalConfig(&config);
    EXPECT_FALSE(pgMigrateFn_gimbalConfig(&config, &old, sizeof(old) - 1, 1));
    EXPECT_FALSE(pgMigrateFn_gimbalConfig(&config, &old, sizeof(old), 7));
    EXPECT_EQ(0, config.modeChannel);
}

TEST(GimbalSerialTest, ModePacketsPreserveAxesSensitivityAndCrc)
{
    const uint8_t expected[3][10] = {
        {0xa5, 0x5a, 0xe8, 0x10, 0x14, 0x72, 0xbd, 0x3d, 0xbd, 0x3f},
        {0xa5, 0x5a, 0xe9, 0x10, 0x14, 0x72, 0xbd, 0x3d, 0xf8, 0x9f},
        {0xa5, 0x5a, 0xea, 0x10, 0x14, 0x72, 0xbd, 0x3d, 0x36, 0x7f},
    };
    for (unsigned mode = 0; mode < 3; ++mode) {
        gimbalHtkAttitudePkt_t packet = {};
        packet.sync[0] = HTKATTITUDE_SYNC0;
        packet.sync[1] = HTKATTITUDE_SYNC1;
        packet.mode = mode;
        packet.sensibility = -3;
        packet.roll = 321;
        packet.tilt = -654;
        packet.pan = 987;
        gimbalSerialFinalizePacket(&packet);
        ASSERT_EQ(10u, sizeof(packet));
        EXPECT_EQ(0, std::memcmp(&packet, expected[mode], 10));
    }
}

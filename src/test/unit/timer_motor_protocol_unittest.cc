/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <array>
#include <vector>
#include <cstring>
extern "C" {
#include "platform.h"
#include "fc/timer_motor_protocol_msp.h"
}
#include "gtest/gtest.h"

namespace {
using Values = std::array<uint8_t, 256>;
Values initial() { Values value; value.fill(255); value[4] = 5; return value; }
bool decode(std::vector<uint8_t> bytes, uint8_t count, const Values &old, Values &out)
{
    // Give empty packets valid backing storage too.
    const size_t length = bytes.size(); bytes.resize(length + 1);
    sbuf_t src; sbufInit(&src, bytes.data(), bytes.data() + length);
    return timerMotorProtocolsDecode(&src, count, old.data(), out.data());
}
void rejected(const std::vector<uint8_t> &bytes, uint8_t count = 18)
{
    auto old = initial(); Values out; out.fill(0xA5); const auto unchanged = out;
    EXPECT_FALSE(decode(bytes, count, old, out)); EXPECT_EQ(out, unchanged);
}
}
TEST(TimerMotorProtocol, ValidPatchPreservesAllUntouchedTimers)
{
    const auto old = initial(); auto out = old;
    ASSERT_TRUE(decode({1, 2, 4, 5, 7, 0}, 18, old, out));
    auto expected = old; expected[7] = 0; EXPECT_EQ(out, expected);
}
TEST(TimerMotorProtocol, DefaultAndEachV1ProtocolRoundTrip)
{
    for (uint8_t protocol : {255, 0, 4, 5, 6}) {
        auto old = initial(); auto out = old;
        ASSERT_TRUE(decode({1, 1, 4, protocol}, 18, old, out));
        EXPECT_EQ(out[4], protocol);
        old[4] = protocol; EXPECT_EQ(out, old);
    }
}
TEST(TimerMotorProtocol, TruncatedTrailingWrongVersionAndPairCountsAreAtomic)
{
    for (const auto &bytes : std::vector<std::vector<uint8_t>> {
        {}, {1}, {0,0}, {2,0}, {255,0}, {1,1}, {1,1,4},
        {1,2,4,5,7}, {1,1,4,5,9}, {1,0,4,5}, {1,19},
        {1,2,4,0,7,254}, {1,2,4,0,18,5}, {1,2,4,0,4,5}
    }) rejected(bytes);
}
TEST(TimerMotorProtocol, EveryInvalidProtocolAndIndexAreRejected)
{
    for (unsigned protocol = 0; protocol < 256; protocol++)
        if (protocol != 255 && protocol != 0 && !(protocol >= 4 && protocol <= 6))
            rejected({1,1,4,static_cast<uint8_t>(protocol)});
    for (unsigned timer = 18; timer < 256; timer++)
        rejected({1,1,static_cast<uint8_t>(timer),5});
}
TEST(TimerMotorProtocol, NoOpAndMaximumCountDoNotWriteBeyondCount)
{
    auto old = initial(); Values out; out.fill(0xA5);
    ASSERT_TRUE(decode({1,0}, 0, old, out));
    for (uint8_t value : out) EXPECT_EQ(value, 0xA5);
    std::vector<uint8_t> packet = {1, 255};
    for (unsigned timer=0; timer<255; timer++) {
        packet.push_back(timer); packet.push_back(timer % 2 ? 0 : 5);
    }
    ASSERT_TRUE(decode(packet, 255, old, out));
    for (unsigned timer=0; timer<255; timer++) EXPECT_EQ(out[timer], timer % 2 ? 0 : 5);
    EXPECT_EQ(out[255], 0xA5);
}
TEST(TimerMotorProtocol, SourceAndDestinationCanAliasForValidAndInvalidPackets)
{
    auto out = initial(); auto before = out;
    ASSERT_TRUE(decode({1,1,7,0}, 18, out, out)); before[7] = 0; EXPECT_EQ(out, before);
    EXPECT_FALSE(decode({1,2,7,5,7,0}, 18, out, out)); EXPECT_EQ(out, before);
}

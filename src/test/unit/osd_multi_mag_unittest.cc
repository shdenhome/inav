/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "sensors/multi_mag.h"
#include "io/osd_multi_mag.h"
}
#include "gtest/gtest.h"
namespace { bool enabled,present[MULTI_MAG_MAX_SOURCES];multiMagSourceState_t sources[MULTI_MAG_MAX_SOURCES]; }
extern "C" {
bool multiMagEnabled(void) { return enabled; }
bool multiMagCaptureSource(unsigned slot,uint32_t,multiMagSourceState_t *source) { if(!present[slot]) return false;*source=sources[slot];return true; }
}
class OsdMultiMagTest:public ::testing::Test { protected:
 void SetUp() override { enabled=false;osdMultiMagUpdate(0);osdMultiMagMessage(0);memset(present,0,sizeof(present));memset(sources,0,sizeof(sources));enabled=true; }
 void source(unsigned i,unsigned trust) { present[i]=true;sources[i].id=i+100;sources[i].flags=MULTI_MAG_CALIBRATED;sources[i].confidence=trust; }
 void expire() { for(unsigned i=1;i<=3;i++) { osdMultiMagSetVisible(true);osdMultiMagMessage(i*1000); } }
};
TEST_F(OsdMultiMagTest, LossOccursOnlyAtZeroAndDoesNotRepeatEachUpdate) {
 source(0,1);osdMultiMagUpdate(0);EXPECT_EQ(osdMultiMagMessage(0),nullptr);
 sources[0].confidence=0;osdMultiMagUpdate(10);EXPECT_STREQ(osdMultiMagMessage(0),"MAG 1 TRUST LOST");
 for(unsigned i=0;i<100;i++)osdMultiMagUpdate(i);
 expire();EXPECT_EQ(osdMultiMagMessage(3001),nullptr);
 sources[0].confidence=1;osdMultiMagUpdate(3002000);EXPECT_STREQ(osdMultiMagMessage(3002),"MAG 1 TRUST RESTORED");
}
TEST_F(OsdMultiMagTest, FallbackRemainsVisibleWithoutPretendingTrustRestored) {
 source(11,0);sources[11].flags|=MULTI_MAG_FALLBACK;osdMultiMagUpdate(0);
 EXPECT_STREQ(osdMultiMagMessage(0),"MAG 12 TRUST LOST FALLBACK");expire();EXPECT_STREQ(osdMultiMagMessage(3001),"MAG 12 FALLBACK");
 osdMultiMagUpdate(3002000);EXPECT_STREQ(osdMultiMagMessage(3002),"MAG 12 FALLBACK");
 enabled=false;osdMultiMagUpdate(4000000);EXPECT_EQ(osdMultiMagMessage(4000),nullptr);
}
TEST_F(OsdMultiMagTest, CalibrationDoesNotGenerateLossAndHiddenEventsDoNotExpire) {
 source(1,0);sources[1].flags|=MULTI_MAG_CALIBRATING;osdMultiMagUpdate(0);EXPECT_EQ(osdMultiMagMessage(0),nullptr);
 sources[1].flags&=~MULTI_MAG_CALIBRATING;osdMultiMagUpdate(100);EXPECT_STREQ(osdMultiMagMessage(10000),"MAG 2 TRUST LOST");
 EXPECT_STREQ(osdMultiMagMessage(20000),"MAG 2 TRUST LOST");
}

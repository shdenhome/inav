/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstdarg>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
extern "C" {
#include "platform.h"
#include "blackbox/multi_mag_log.h"
}
#include "gtest/gtest.h"
namespace { std::vector<uint8_t> bytes; multiMagSourceState_t sources[MULTI_MAG_MAX_SOURCES]; bool present[MULTI_MAG_MAX_SOURCES]; }
extern "C" {
int32_t blackboxHeaderBudget;
void blackboxWrite(uint8_t value) { bytes.push_back(value); }
int blackboxPrint(const char *) { return 0; }
int tfp_format(void *,void (*)(void *,char),const char *,va_list) { return 0; }
bool multiMagCaptureSource(unsigned slot,uint32_t,multiMagSourceState_t *source) { if(!present[slot]) return false;*source=sources[slot];return true; }
}
class MultiMagLogTest:public ::testing::Test { protected:
 void SetUp() override { bytes.clear();memset(sources,0,sizeof(sources));memset(present,0,sizeof(present)); }
};
TEST_F(MultiMagLogTest, QuantizesAllPhysicalSlotsAndPreservesPositiveTrust) {
 for(unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) { present[i]=true;sources[i].aligned[0]=-2032;sources[i].aligned[1]=2032;sources[i].confidence=i?1000:1;sources[i].flags=MULTI_MAG_READABLE|MULTI_MAG_FRESH|MULTI_MAG_CONTRIBUTING|MULTI_MAG_CALIBRATED; }
 multiMagLogState_t state;multiMagLogCapture(&state,1000);
 for(unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) { EXPECT_EQ(state.value[i][0],-127);EXPECT_EQ(state.value[i][1],127);EXPECT_EQ(state.value[i][3]&255,i?255:1);EXPECT_EQ(state.value[i][3]>>8,11); }
 multiMagLogWrite(&state,0xfff);EXPECT_LE(bytes.size(),72u);
}
TEST_F(MultiMagLogTest, ZeroTrustFallbackIsExplicitAndInvalidValuesAreFlagged) {
 present[8]=true;sources[8].flags=MULTI_MAG_READABLE|MULTI_MAG_FRESH|MULTI_MAG_FALLBACK|MULTI_MAG_CONTRIBUTING|MULTI_MAG_CALIBRATED;
 sources[8].aligned[0]=1e20f;sources[8].aligned[1]=-1e20f;sources[8].aligned[2]=std::numeric_limits<float>::quiet_NaN();
 multiMagLogState_t state;multiMagLogCapture(&state,1000);
 EXPECT_EQ(state.value[8][0],127);EXPECT_EQ(state.value[8][1],-127);EXPECT_EQ(state.value[8][2],0);
 EXPECT_EQ(state.value[8][3]&255,0);EXPECT_EQ(state.value[8][3]>>8,47);
 EXPECT_EQ(state.value[0][3],0);
}
TEST_F(MultiMagLogTest, AbsoluteSnapshotsRemainBoundedAndAbsentMaskIsEmpty) {
 multiMagLogState_t state{};
 for(unsigned i=0;i<MULTI_MAG_MAX_SOURCES;i++) for(unsigned j=0;j<4;j++) { state.value[i][j]=j==3?16383:127; }
 multiMagLogWrite(&state,0xfff);EXPECT_LE(bytes.size(),72u);
 bytes.clear();multiMagLogWrite(&state,0xfff);EXPECT_LE(bytes.size(),72u);
 bytes.clear();multiMagLogWrite(&state,0);EXPECT_TRUE(bytes.empty());
}

#include <fstream>
#include <sstream>
#include <cstdlib>
extern "C" {
#include "blackbox/blackbox_encoding.h"
}
TEST_F(MultiMagLogTest, StandardDecoderFixtureHasEverySourceInEveryMainFrame) {
 std::ostringstream log;log<<"H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\nH Firmware type:Cleanflight\nH Firmware revision:INAV 10.0.0 (validation) MATEKH743HD\nH I interval:32\nH P interval:1/1\nH acc_1G:2048\nH gyro_scale:0x3f800000\nH features:0\nH vbatref:0\n";
 log<<"H Field I name:loopIteration,time\nH Field I signed:0,0\nH Field I predictor:0,0\nH Field I encoding:1,1\nH Field P predictor:6,2\nH Field P encoding:9,0\nH Field S name:";
 bool comma=false;
 for(unsigned i=0;i<12;i++) for(auto name:{"magSourceX","magSourceY","magSourceZ","magSourceQuality"}) { if(comma)log<<',';comma=true;log<<name<<'['<<i<<']'; }
 for(auto key:{"signed","predictor","encoding"}) { log<<"\nH Field S "<<key<<':';for(unsigned i=0;i<48;i++){if(i)log<<',';log<<(strcmp(key,"signed")==0?1:strcmp(key,"encoding")==0?8:0);} }log<<'\n';
 multiMagLogState_t state{};
 for(unsigned frame=0;frame<5;frame++) {
  for(unsigned i=0;i<12;i++) { state.value[i][0]=-100+int(frame);state.value[i][1]=i;state.value[i][2]=64;state.value[i][3]=frame<2?3071:256*47; }
  bytes.clear();blackboxWrite('S');multiMagLogWrite(&state,0xfff);EXPECT_LE(bytes.size(),73u);
  log.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());bytes.clear();blackboxWrite(frame?'P':'I');
  if(!frame){blackboxWriteUnsignedVB(0);blackboxWriteUnsignedVB(1000000);}else blackboxWriteSignedVB(frame==1?1000:0);
  log.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
 }
 if(const char *path=std::getenv("INAV_TEST_MAG_LOG_FIXTURE")) { std::ofstream file(path,std::ios::binary);ASSERT_TRUE(file.is_open());file<<log.str();ASSERT_TRUE(file.good()); }
}

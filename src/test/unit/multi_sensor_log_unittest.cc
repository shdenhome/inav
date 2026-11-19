/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstdarg>
#include <cstring>
#include <vector>
#include <limits>
#include <fstream>
#include <sstream>
#include <cstdlib>
extern "C" {
#include "platform.h"
#include "blackbox/multi_sensor_log.h"
#include "blackbox/blackbox_encoding.h"
#include "blackbox/blackbox_fielddefs.h"
#include "sensors/multi_sensor_telemetry.h"
}
#include "gtest/gtest.h"
namespace {
std::vector<uint8_t> written;
multiSensorTelemetry_t snapshot;
struct Field { const char *name; unsigned index, offset, group; };
#define FIELD(name, index, member, group) {#name, index, offsetof(multiSensorLogState_t, member), group},
const Field mainFields[] = { MULTI_SENSOR_MAIN_LOG_FIELDS(FIELD) };
const Field slowFields[] = { MULTI_SENSOR_SLOW_LOG_FIELDS(FIELD) };
#undef FIELD
int32_t value(const multiSensorLogState_t &state, const Field &field) {
    int32_t result;memcpy(&result,reinterpret_cast<const uint8_t*>(&state)+field.offset,sizeof(result));return result;
}
int32_t readSigned(unsigned &offset) {
    uint32_t encoded=0;unsigned shift=0;uint8_t byte;
    do {
        if(offset>=written.size()||shift>=35) { ADD_FAILURE()<<"truncated signed VB";return 0; }
        byte=written[offset++];encoded|=uint32_t(byte&0x7f)<<shift;shift+=7;
    } while(byte&0x80);
    return static_cast<int32_t>((encoded>>1)^uint32_t(-int32_t(encoded&1)));
}
}
extern "C" {
int32_t blackboxHeaderBudget;
void blackboxWrite(uint8_t value) { written.push_back(value); }
int blackboxPrint(const char *) { return 0; }
int tfp_format(void *,void (*)(void *,char),const char *,va_list) { return 0; }
void multiSensorCaptureTelemetry(multiSensorTelemetry_t *state,uint32_t now) { *state=snapshot;state->timestampUs=now; }
}
class MultiSensorLogTest : public ::testing::Test {
protected:
    multiSensorLogState_t state{};
    void SetUp() override {
        written.clear();memset(&snapshot,0,sizeof(snapshot));
        for(unsigned i=0;i<3;i++) {
            auto &imu=snapshot.imu[i];imu.hardware=i==1?3:8;imu.flags=127;imu.gyroTrust=900;imu.accTrust=800;
            imu.gyroAgeUs=400;imu.accAgeUs=1200;
            for(unsigned axis=0;axis<3;axis++) { imu.gyroMdps[axis]=int(i*3+axis)*1000-5000;imu.accMg[axis]=int(axis)*100-100;imu.rawAccMg[axis]=int(axis)*120-120; }
        }
        for(unsigned i=0;i<2;i++) {
            auto &baro=snapshot.baro[i];baro.hardware=i==0?3:7;baro.flags=15;baro.trust=990;baro.ageUs=12000;
            baro.pressurePa=100000+i;baro.alignedPressurePa=99930+i;baro.temperatureCenti=-1250;baro.altitudeCm=-123;
        }
        multiSensorLogCapture(&state,100000);
    }
    template<size_t N> void checkFrame(const Field (&fields)[N],const multiSensorLogState_t *previous,unsigned groups) {
        unsigned cursor=0;
        for(const auto &field:fields) if(field.group&groups) {
            int32_t decoded=readSigned(cursor);
            if(previous) decoded=static_cast<int32_t>(uint32_t(decoded)+uint32_t(value(*previous,field)));
            EXPECT_EQ(decoded,value(state,field))<<field.name<<'['<<field.index<<']';
        }
        EXPECT_EQ(cursor,written.size());
    }
};
TEST_F(MultiSensorLogTest, HeaderFieldOrderAndIntraframeRoundTripMatchPhysicalSources) {
    EXPECT_EQ(sizeof(mainFields)/sizeof(mainFields[0]),20u);
    EXPECT_EQ(sizeof(slowFields)/sizeof(slowFields[0]),41u);
    EXPECT_EQ(state.imuType[1],3);EXPECT_EQ(state.imuAccMg[4],0);EXPECT_EQ(state.baroTempCenti[0],-1250);
    multiSensorLogWriteMain(&state,nullptr,15);checkFrame(mainFields,nullptr,15);
}
TEST_F(MultiSensorLogTest, InterframeUsesPreviousPredictorForEveryAppendedField) {
    multiSensorLogState_t previous=state;
    state.imuGyroMdps[3]=-123456;state.imuAccMg[8]=1020;state.baroPressurePa[1]=98500;
    multiSensorLogWriteMain(&state,&previous,15);checkFrame(mainFields,&previous,15);
}
TEST_F(MultiSensorLogTest, StandardSlowFrameIsIndependentAndCarriesTrustFlagsAndAge) {
    snapshot.imu[1].gyroAgeUs=UINT32_MAX;multiSensorLogCapture(&state,200000);
    EXPECT_EQ(state.imuGyroAgeMs[1],65535);
    state.imuFlags[1]=1;state.imuAccTrust[1]=0;
    multiSensorLogWriteSlow(&state,15);checkFrame(slowFields,nullptr,15);
    EXPECT_LT(written.size()+80,256u); // conservative space for legacy S fields
}
TEST_F(MultiSensorLogTest, ModesAndExistingAccelerometerSelectionFilterBothHeadersAndData) {
    for(unsigned groups : {0u,1u,2u,4u,8u,3u,11u,15u}) {
        written.clear();multiSensorLogWriteMain(&state,nullptr,groups);checkFrame(mainFields,nullptr,groups);
        written.clear();multiSensorLogWriteSlow(&state,groups);checkFrame(slowFields,nullptr,groups);
    }
}
TEST_F(MultiSensorLogTest, ExtremeSignedDeltaHasNoUndefinedOverflow) {
    multiSensorLogState_t previous=state;previous.imuGyroMdps[0]=INT32_MIN;state.imuGyroMdps[0]=INT32_MAX;
    multiSensorLogWriteMain(&state,&previous,15);checkFrame(mainFields,&previous,15);
}

TEST_F(MultiSensorLogTest, EmitsStandardDecoderFixtureWithMainAndSlowFieldHeaders) {
    std::ostringstream log;
    log << "H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\n"
        << "H Firmware type:Cleanflight\nH Firmware revision:INAV 10.0.0 (validation) MATEKH743HD\n"
        << "H I interval:32\nH P interval:1/1\nH acc_1G:2048\nH gyro_scale:0x3f800000\nH features:0\nH vbatref:0\n";
    log << "H Field I name:loopIteration,time";
    for(const auto &f:mainFields) log << ',' << f.name << '[' << f.index << ']';
    log << "\nH Field I signed:0,0";for(const auto &f:mainFields) { (void)f;log<<",1"; }
    log << "\nH Field I predictor:0,0";for(const auto &f:mainFields) { (void)f;log<<",0"; }
    log << "\nH Field I encoding:1,1";for(const auto &f:mainFields) { (void)f;log<<",0"; }
    log << "\nH Field P predictor:" << FLIGHT_LOG_FIELD_PREDICTOR_INC << ',' << FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE;
    for(const auto &f:mainFields) { (void)f;log<<",1"; }
    log << "\nH Field P encoding:" << FLIGHT_LOG_FIELD_ENCODING_NULL << ",0";
    for(const auto &f:mainFields) { (void)f;log<<",0"; }
    log << "\nH Field S name:activeWpNumber,flightModeFlags,flightModeFlags2,activeFlightModeFlags,stateFlags";
    for(const auto &f:slowFields) log<<','<<f.name<<'['<<f.index<<']';
    for(const auto *key:{"signed","predictor","encoding"}) {
        log << "\nH Field S " << key << ':';
        for(unsigned i=0;i<5;i++) { if(i)log<<',';log<<(strcmp(key,"encoding")==0?1:0); }
        for(const auto &f:slowFields) { (void)f;log<<','<<(strcmp(key,"signed")==0?1:0); }
    }
    log << '\n';
    multiSensorLogState_t previous=state;
    for(unsigned i=0;i<5;i++) {
        if(i==0||i==2) {
            if(i==2) state.imuAccTrust[1]=125;
            written.clear();blackboxWrite('S');
            for(unsigned n=0;n<5;n++) blackboxWriteUnsignedVB(0);
            multiSensorLogWriteSlow(&state,15);
            EXPECT_LT(written.size(),256u);
            log.write(reinterpret_cast<const char*>(written.data()),written.size());
        }
        state.imuGyroMdps[0]=-5000+int(i);
        written.clear();blackboxWrite(i==0?'I':'P');
        if(i==0) { blackboxWriteUnsignedVB(0);blackboxWriteUnsignedVB(1000000); }
        else blackboxWriteSignedVB(i==1?1000:0);
        multiSensorLogWriteMain(&state,i==0?nullptr:&previous,15);
        EXPECT_LT(written.size()+100,256u);
        log.write(reinterpret_cast<const char*>(written.data()),written.size());
        previous=state;
    }
    if(const char *path=std::getenv("INAV_TEST_LOG_FIXTURE")) {
        std::ofstream file(path,std::ios::binary);ASSERT_TRUE(file.is_open());file<<log.str();ASSERT_TRUE(file.good());
    }
    EXPECT_NE(log.str().find("imuAccTrust[1]"),std::string::npos);
}

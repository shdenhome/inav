/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>
extern "C" {
#include "platform.h"
#include "drivers/time.h"
#include "sensors/battery.h"
uint16_t adcGetChannel(uint8_t) { return 0; }
bool adcIsFunctionAssigned(uint8_t) { return false; }
timeMs_t millis(void) { return 0; }
}
#include "gtest/gtest.h"
TEST(Battery2InitTest, PersistedAdcAndI2cCalibrationsRemainByteForByteUnchanged) {
    for(unsigned type:{BATTERY2_SENSOR_ADC,BATTERY2_SENSOR_I2C}) {
        for(unsigned voltageScale:{1000,2100,1234}) {
            for(unsigned currentScale:{2,100,250,65535}) {
                auto *config=battery2ConfigMutable();memset(config,0,sizeof(*config));
                config->voltageEnabled=config->currentEnabled=1;
                config->voltageSensor=config->currentSensor=type;
                config->voltageScale=voltageScale;config->currentScale=currentScale;
                config->voltageOffset=-37;config->currentOffset=19;
                config->currentI2cAddress=69;config->cells=6;
                config->cellDetect=430;config->cellWarning=350;config->cellCritical=330;
                const battery2Config_t previous=*config;
                battery2Init();
                EXPECT_EQ(memcmp(config,&previous,sizeof(previous)),0)
                    <<"type="<<type<<" voltage="<<voltageScale<<" current="<<currentScale;
                battery2Init(); // Re-initialization must preserve it too.
                EXPECT_EQ(memcmp(config,&previous,sizeof(previous)),0);
            }
        }
    }
}

#!/usr/bin/env python3
"""Exercise the live shared MSP setter with stub PG/arming/validation providers."""
from pathlib import Path
import os,re,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[3]
source=(ROOT/'src/main/fc/fc_msp.c').read_text()
start=source.index('static bool mspSetMotorProtocol(')
open_at=source.index('{',start)
depth=0
for pos in range(open_at,len(source)):
    if source[pos]=='{': depth+=1
    if source[pos]=='}':
        depth-=1
        if depth==0:
            body=source[start:pos+1];break
assert source.count('mspSetMotorProtocol(')==3, 'Both MSP setter call sites must use guard'
assert 'ptr == &motorConfigMutable()->motorPwmProtocol' in source
prelude=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#define TIMER_MOTOR_PROTOCOL_DEFAULT 255
#define PWM_TYPE_SRXL2 7
#define ARMED 1
#define ARMING_FLAG(x) armed
static bool armed,valid;
static int calls;
static unsigned seenProtocol;
typedef struct {uint8_t motorPwmProtocol;} motorConfig_t;
typedef struct {uint8_t protocol;} timerMotorProtocolConfig_t;
static motorConfig_t config;
static timerMotorProtocolConfig_t overrides[14];
static uint8_t observed[14];
static motorConfig_t *motorConfigMutable(void){return &config;}
static const motorConfig_t *motorConfig(void){return &config;}
static const timerMotorProtocolConfig_t *timerMotorProtocols(unsigned i){return &overrides[i];}
static bool pwmTimerMotorProtocolsValidate(const uint8_t *p){calls++;seenProtocol=config.motorPwmProtocol;memcpy(observed,p,14);return valid;}
'''
tests=r'''
int main(void){
    for(unsigned proposed=0;proposed<256;proposed++){
        memset(overrides,255,sizeof(overrides));config.motorPwmProtocol=5;valid=true;armed=false;calls=0;
        assert(mspSetMotorProtocol(proposed)==(proposed<=7));
        assert(config.motorPwmProtocol==(proposed<=7?proposed:5));assert(calls==0);
        config.motorPwmProtocol=5;armed=true;
        assert(!mspSetMotorProtocol(proposed));assert(config.motorPwmProtocol==5);assert(calls==0);
    }
    armed=false;overrides[7].protocol=0;config.motorPwmProtocol=5;valid=false;calls=0;
#if !defined(SITL_BUILD) && !defined(RP2350)
    assert(!mspSetMotorProtocol(4));assert(config.motorPwmProtocol==5);
    assert(calls==1);assert(seenProtocol==4);assert(observed[7]==0);
    valid=true;assert(mspSetMotorProtocol(4));assert(config.motorPwmProtocol==4);
    assert(calls==2);assert(seenProtocol==4);assert(observed[7]==0);
#else
    assert(mspSetMotorProtocol(4));assert(config.motorPwmProtocol==4);assert(calls==0);
#endif
    puts("production shared MSP guard checks passed");
}
'''
with tempfile.TemporaryDirectory(prefix='inav-msp-protocol-guard-') as tmp:
    src=Path(tmp)/'guard.c';src.write_text(prelude+'\n'+body+'\n'+tests)
    for name,defs in [('stm32',[]),('sitl',['SITL_BUILD']),('rp2350',['RP2350'])]:
        binary=Path(tmp)/name
        subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-O1','-g','-fsanitize=address,undefined',*[f'-D{x}' for x in defs],str(src),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True);print(name+': passed',flush=True)

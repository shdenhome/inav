#!/usr/bin/env python3
"""Run real pwm_mapping.c policy bodies with fake hardware, never copied policy.

This checks configuration policy only. It does not simulate timer/DMA electrical
behaviour. Compile each feature variant so no-DSHOT and BRUSHED regressions fail.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'src/main/drivers/pwm_mapping.c').read_text()
header = (ROOT / 'src/main/drivers/pwm_mapping.h').read_text()

def function(name):
    match = re.search(r'^[^\n;{}]*\b' + name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise RuntimeError('Production function missing: ' + name)
    # Ignore comment/string delimiters while matching nested function braces.
    tail = source[match.start():]
    clean = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"',
                   lambda m: ' ' * len(m[0]), tail, flags=re.S)
    depth = 0
    for i, char in enumerate(clean):
        if char == '{': depth += 1
        if char == '}':
            depth -= 1
            if depth == 0: return tail[:i+1]
    raise RuntimeError('Unterminated production function: ' + name)

protocol_enum = re.search(r'typedef enum \{\s*PWM_TYPE_STANDARD.*?\} motorPwmProtocolTypes_e;', header, re.S)[0]
properties = re.search(r'static const motorProtocolProperties_t motorProtocolProperties\[\] = \{.*?\n\};', source, re.S)[0]
prelude = r'''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#define TIMER_MOTOR_PROTOCOL_DEFAULT 255
#define TIMER_HW_MAX 64
#define FEATURE_REVERSIBLE_MOTORS 1
#define MAX_SUPPORTED_MOTORS 12
#define MAX_MOTORS 12
#define MAX_MIXER_PROFILE_COUNT 2
#define UNUSED(x) (void)(x)
#define ARRAYLEN(x) (sizeof(x)/sizeof((x)[0]))
typedef struct { bool usesHwTimer; bool isDSHOT; } motorProtocolProperties_t;
typedef struct { unsigned tim; unsigned dmaTag; } timerHardware_t;
typedef struct { unsigned outputMode; } timerOverride_t;
typedef struct { int maxTimMotorCount; int maxTimServoCount; const timerHardware_t *timMotors[32]; } timMotorServoHardware_t;
typedef struct { unsigned motorPwmProtocol; } motorConfig_t;
static motorConfig_t config;
static timerOverride_t modes[14];
static timerHardware_t timerHardware[80];
static int timerHardwareCount;
static int motorCount;
static bool reversible;
typedef struct {float throttle;} motorMixer_t;
static motorMixer_t rules[2][12];
static const motorMixer_t *mixerMotorMixersByIndex(unsigned profile) {return rules[profile];}
static timMotorServoHardware_t assignment;
static const motorConfig_t *motorConfig(void) { return &config; }
static unsigned timer2id(unsigned tim) { return tim; }
static void *dmaGetByTag(unsigned tag) { return tag ? &config : NULL; }
static const timerOverride_t *timerOverrides(unsigned i) { return &modes[i]; }
static int getMotorCount(void) { return motorCount; }
static bool featureConfigured(unsigned f) { (void)f; return reversible; }
static void pwmCalculateAssignmentForMotorCount(timMotorServoHardware_t *out, const uint8_t *proposed, uint8_t count) { (void)proposed; assert(count==5); *out = assignment; }
'''
tests = r'''
static void setup(unsigned global) {
    memset(timerHardware,0,sizeof(timerHardware)); memset(&assignment,0,sizeof(assignment));
    memset(rules,0,sizeof(rules)); for(int i=0;i<5;i++)rules[i%2][i].throttle=1;
    config.motorPwmProtocol=global; reversible=false; timerHardwareCount=5; motorCount=5;
    for(int i=0;i<5;i++) {
        timerHardware[i].tim=i<4?4:7; timerHardware[i].dmaTag=i+1;
        assignment.timMotors[i]=&timerHardware[i];
    }
    assignment.maxTimMotorCount=5;
}
int main(void) {
    uint8_t p[14]; memset(p,255,sizeof(p));
    setup(PWM_TYPE_STANDARD); assert(pwmTimerMotorProtocolsValidate(p));
    assert(pwmConfiguredMotorCount()==5);
    motorCount=1; assert(pwmConfiguredMotorCount()==5); // runtime count is deliberately independent
    rules[1][5].throttle=1; assert(pwmConfiguredMotorCount()==6); rules[1][5].throttle=0;
    rules[0][2].throttle=0; assert(pwmConfiguredMotorCount()==2); rules[0][2].throttle=1;
    assert(!pwmTimerMotorProtocolsValidate(NULL));
    assert(pwmTimerMotorProtocolMask(13)==0);
#ifdef BRUSHED_MOTORS
    assert(pwmTimerMotorProtocolMask(4)==0);
    p[4]=PWM_TYPE_STANDARD; assert(!pwmTimerMotorProtocolsValidate(p));
#else
    assert(pwmTimerMotorProtocolMask(4)&(1<<PWM_TYPE_STANDARD));
    p[4]=PWM_TYPE_STANDARD; assert(pwmTimerMotorProtocolsValidate(p));
    config.motorPwmProtocol=254; assert(!pwmTimerMotorProtocolsValidate(p)); config.motorPwmProtocol=PWM_TYPE_STANDARD;
    timerHardwareCount=65; assert(!pwmTimerMotorProtocolsValidate(p)); timerHardwareCount=5;
    p[13]=PWM_TYPE_STANDARD; assert(!pwmTimerMotorProtocolsValidate(p)); p[13]=255;
    p[4]=254; assert(!pwmTimerMotorProtocolsValidate(p));
    p[4]=PWM_TYPE_ONESHOT125; assert(!pwmTimerMotorProtocolsValidate(p));
#ifdef USE_DSHOT
    p[4]=PWM_TYPE_DSHOT300; p[7]=PWM_TYPE_STANDARD;
    assert(pwmTimerMotorProtocolsValidate(p));
    reversible=true; assert(!pwmTimerMotorProtocolsValidate(p)); reversible=false;
    p[7]=PWM_TYPE_DSHOT600; assert(!pwmTimerMotorProtocolsValidate(p));
    p[7]=PWM_TYPE_DSHOT300; assert(pwmTimerMotorProtocolsValidate(p));
    config.motorPwmProtocol=PWM_TYPE_SRXL2; assert(!pwmTimerMotorProtocolsValidate(p));
    config.motorPwmProtocol=PWM_TYPE_STANDARD;
    assignment.maxTimMotorCount=4; assert(!pwmTimerMotorProtocolsValidate(p)); assignment.maxTimMotorCount=5;
    timerHardware[2].dmaTag=0; assert(!(pwmTimerMotorProtocolMask(4)&(1<<PWM_TYPE_DSHOT300)));
    assert(!pwmTimerMotorProtocolsValidate(p)); timerHardware[2].dmaTag=3;
    // Mix forbidden inherited analog protocols in either logical motor order.
    config.motorPwmProtocol=PWM_TYPE_MULTISHOT; p[7]=255; assert(!pwmTimerMotorProtocolsValidate(p));
    p[4]=255; p[7]=PWM_TYPE_DSHOT300; assert(!pwmTimerMotorProtocolsValidate(p));
#else
    assert(!(pwmTimerMotorProtocolMask(4)&(1<<PWM_TYPE_DSHOT300)));
    p[4]=PWM_TYPE_DSHOT300; assert(!pwmTimerMotorProtocolsValidate(p));
#endif
#endif
    puts("production timer policy checks passed");
}
'''
unit = '\n'.join([prelude, protocol_enum, properties,
    function('getMotorProtocolProperties'), function('pwmConfiguredMotorCount'), function('motorsUseHardwareTimers'),
    function('pwmTimerMotorProtocolMask'), function('pwmTimerMotorProtocolsValidate'), tests])
with tempfile.TemporaryDirectory(prefix='inav-timer-policy-') as tmp:
    src = Path(tmp)/'policy.c'; src.write_text(unit)
    for name,defines in [('dshot',['USE_DSHOT']), ('analog',[]), ('brushed',['BRUSHED_MOTORS']), ('brushed_dshot',['BRUSHED_MOTORS','USE_DSHOT'])]:
        binary=Path(tmp)/name
        subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-O1','-g','-fsanitize=address,undefined',*[f'-D{x}' for x in defines],str(src),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
        print(name + ': passed',flush=True)

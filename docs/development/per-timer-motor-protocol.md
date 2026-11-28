# Per-timer motor protocol (experimental v1)

## Decision and scope

The mixer continues producing the existing motor command domain. Conversion to
DShot units is per initialized motor; driver contexts select the writer. A timer
shares a timebase, so configuration belongs to the hardware timer (zero-based
`timer2id`, TIM5 is 4), not a logical motor or mixer profile. This is a
platform-defined packed index, not generally TIM number minus one (for example,
higher-numbered H7 timers can be packed into lower indices). Use the board
output mapping to identify a timer. Timers are initialized
at boot and are never reconfigured by profile switching or MSP writes.

Keep `motor_pwm_protocol` as a real protocol and default. `MIXED` is a derived
Configurator display, never a firmware enum. Explicit overrides in v1 are
`STANDARD`, `DSHOT150`, `DSHOT300`, `DSHOT600`, and `DEFAULT` (255). Protocol
numeric values remain unchanged. The mixed combination supported by v1 is
STANDARD plus one DShot speed. Mixed DShot speeds and other mixed analog
protocols are rejected, pending independent scheduling/validation. Reversible
mixed operation is rejected because its legacy conversion depends on a global
protocol. Existing uniform inherited configurations retain their legacy behavior.

## Persistence and backwards compatibility

Use a separate master PG `PG_TIMER_MOTOR_PROTOCOL_CONFIG` (1051, version 0)
containing one protocol byte per hardware timer. Reset initializes every entry
to DEFAULT. The existing `PG_TIMER_OVERRIDE_CONFIG` (1032) and its version/size
remain unchanged. On upgrade older images have no new PG; only the new PG resets,
so motor/servo assignments, old global protocol and both mixer profiles survive.
The PG is shared by both mixer profiles. No automatic EEPROM write occurs.
Old Configurators retain the legacy timer output API and cannot erase new
protocol bytes through those messages. A downgrade image does not know the new
PG and may discard it when saving; make a dump before downgrading.

## CLI

```
set motor_pwm_protocol = DSHOT300
timer_motor_protocol 4 DSHOT300
timer_motor_protocol 7 STANDARD
save
```

`timer_motor_protocol` lists all stored overrides; an index alone queries one.
`timer_motor_protocol 7 DEFAULT` removes an override. `dump`/`diff`, including
`all`/`showdefaults`, use the existing PG backup/default comparison mechanism.
Indices and names are parsed strictly. Complete configuration is validated at
boot: intermediate states while replaying a dump can be temporarily incompatible.
An invalid final configuration must fail motor initialization and block arming,
without silently substituting another protocol.

## Experimental MSP v1 wire contract

New IDs are branch-local and must be coordinated with upstream before merging.
They were checked against the command catalog in this checkout; existing command
IDs and binary layouts are untouched. Bytes are unsigned; U16 is little endian.

* `0x20A5 MSP2_INAV_TIMER_MOTOR_PROTOCOLS`, request empty. Reply:
  `version=1:u8, count:u8`, then `count` records
  `{timer:u8, configured:u8, resolvedConfigured:u8}`. All hardware definition
  indices are included. DEFAULT resolves against the current RAM global protocol.
  This is configuration state, **not** a claim that a live timer was reinitialized;
  save and reboot is required before the physical outputs change.
* `0x20A6 MSP2_INAV_SET_TIMER_MOTOR_PROTOCOLS`, request:
  `version=1:u8, count:u8`, then `count` pairs `{timer:u8, protocol:u8}`.
  Exact length required; unknown version, duplicate timer, invalid index/protocol,
  armed state or an incompatible resulting configuration yields MSP error with
  no partial settings mutation. Omitted timers retain their values. A zero count
  is a valid no-op if the current complete setup validates. Success response is
  exactly `[1,0]`. Values are stored in RAM only. The client must verify this
  explicit success response before proceeding to EEPROM write/reboot.
* `0x20A7 MSP2_INAV_TIMER_PROTOCOL_CAPABILITIES`, request empty. Reply:
  `version=1:u8, count:u8`, records `{timer:u8, allowedProtocolMask:u16}`.
  A bit is the existing protocol enum number. DEFAULT is implicit; absent physical
  timers have mask 0. These are intrinsic timer/build capabilities. A bit does
  **not** guarantee DMA is free with the whole current resource configuration.
  Full set validation and boot initialization can still reject conflicts.
  V1 conservatively advertises DShot only when every physical pad of that timer
  has a DMA route, including unused pads. A group with an unused DMA-less channel
  may therefore be unavailable even if a smaller subset could work electrically.

The commands are unavailable on SITL and RP2350 until their platform drivers
implement the same contracts. Clients probe the new API and hide controls for
unsupported/invalid replies rather than guessing support from firmware version.
The legacy advanced-config setter rejects changing the global protocol to an
incompatible override combination, restoring its old value on rejection.

## Runtime resource and mapping notes

Pending configuration validation computes the motor union from both stored mixer
profiles, independently of the boot-latched runtime motor count. Assignment
simulation starts from saved board timer usage flags and restores live flags on
exit. No live timer or motor context is reinitialized while editing configuration.
The flags snapshot adds 256 bytes of static RAM; on 32-bit ARM the per-motor
protocol and writer add approximately eight bytes per motor, plus a few global
state bytes and the new PG. Actual linked RAM/flash totals and runtime CPU load
remain to be measured in requested builds and hardware tests.

Occupied DMA routes fail initialization before being reused by another timer.
Any motor/servo initialization failure disables motor writes and digital updates
and clears partially initialized motor compare registers. Mixed mode exposes
only DShot slots to DShot commands/telemetry and mixed-mode ESC passthrough;
uniform analog passthrough retains its legacy behavior. Turtle mode requires
all initialized motor outputs to be DShot.

## Validation and remaining hardware acceptance

Decoder unit tests exercise strict framing, version, every possible protocol byte,
invalid/duplicate indices and atomic failure. The driver must check every DShot
DMA path, telemetry request and command against the initialized motor type.
No package build or physical signal validation is implied by source tests.
Before release use an oscilloscope/logic analyzer on F4/F7/H7 to confirm the
mandatory DSHOT300 + STANDARD scenario, ordinary uniform PWM/DShot, disarm,
failsafe, individual motor testing, ESC telemetry, passthrough, LED/servo DMA
conflicts and both VTOL profiles. Verify old EEPROM upgrade, dump replay and
new/old Configurator interoperability. Hardware acceptance remains pending.

# Per-timer motor protocols: verification plan

This feature changes the motor signal driver. Host tests do not establish physical
signal correctness. No hardware acceptance is implied by passing software checks.

## Automated protocol checks

`timer_motor_protocol_unittest` exercises the production MSP decoder rather than a
copied implementation. It covers valid patches, DEFAULT and each V1 protocol,
unchanged timers, invalid versions, truncated/trailing bytes, duplicate timer IDs,
invalid timer IDs and protocol bytes, failure atomicity, aliased input/output, and
zero/maximum record counts. Capability/resource validation is a separate layer.

`python3 src/test/support/check_timer_motor_protocol_policy.py` extracts the live
production mask/validation function bodies and compiles them with stub hardware,
assignment and feature providers. It does not copy the policy implementation.
Four compile variants (DShot, analogue-only, brushed, brushed plus DShot) run with
`-Wall -Wextra -Werror` and AddressSanitizer/UndefinedBehaviorSanitizer. Cases cover
the mandatory four-DShot/one-PWM configuration, inheritance, supported uniform
setups, unsupported mixed rates/reversal/analogue protocols, missing DMA, missing
timers, invalid protocol bytes, oversized hardware tables and output capacity.
This verifies policy, not timer initialization or physical DMA behavior.

## Bench acceptance (pending)

Use a fixed test rig with propellers removed and a logic analyser/oscilloscope.
Record the exact firmware revision, target, timer assignment, ESC models and
configuration with each capture. Do not treat the absence of a motor-init error as
proof that the captured waveform is correct.

| Scenario | Required evidence |
| --- | --- |
| All DEFAULT, STANDARD | Same output assignment, pulse widths and frequency as baseline |
| All DEFAULT, DSHOT300 | Same logical motor order, digital packets and rate as baseline |
| TIM5 DSHOT300 + TIM8 STANDARD | Four correct DShot channels and one PWM channel concurrently |
| Mixed plus servos | Servo frequency/pulse widths unaffected; no shared time-base conflict |
| Mixed plus LED | LED DMA and motor DMA both work; unavailable resources give an explicit init error |
| Digital command/beeper | Commands appear only on digital motor pins; analogue PWM unchanged |
| ESC telemetry | Requests skip analogue motors and returned data stays associated with the correct logical motor |
| Motor test | Each slider drives only its assigned motor through the correct protocol |
| Disarm, motor stop, failsafe | Every motor reaches its configured stop condition through its own protocol |
| VTOL profile switch/transition | Motor mapping and timer initialisation stay fixed; correct outputs in both profiles |
| Save and reboot | Every override restored and applied; DEFAULT resolves to the global protocol |
| Dump/diff restore | Same overrides, mode assignments and motor order after full replay |
| Old stored configuration | Existing timer output modes survive; new overrides initialise to DEFAULT |
| Old Configurator | Original command layouts unchanged; old saves do not clear override PG |
| New Configurator, old firmware | Unsupported API hidden; no stalled tab load or unsolicited overwrite |
| Invalid DMA/protocol combination | Explicit error and arming disabled; no automatic fallback |
| Unsupported V1 combinations | Different DShot rates/reversible mixed setup are rejected, not silently approximated |
| Flash write while disarmed | Digital zero-throttle keepalive present and analogue outputs remain safe |
| CPU/RAM | Before/after timing, resource utilisation and worst-case jitter captured |

At minimum repeat the relevant hardware scenarios on STM32F4, STM32F7 and STM32H7.
A target family not tested is not hardware-validated by a successful compiler run.

## Independent review focus

- Protocol conversion belongs to each logical motor after mixing.
- No analogue motor reaches DShot command, DMA or telemetry code.
- The last motor slot and sparse mixed assignments are included in all loops.
- Configuration changes alter RAM only until the normal Save/reboot flow.
- New settings do not discard old timer-mode assignments during PG reset.
- Capabilities describe hardware support; the complete final combination is
  validated again because individually available DMA routes can conflict.
- Cached runtime protocol cannot change underneath already-initialised timers.
- MIXED is derived UI state, never a new low-level protocol enum.

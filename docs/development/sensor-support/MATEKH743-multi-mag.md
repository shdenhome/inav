# MATEKH743HD multiple I2C magnetometers

Technical design and acceptance record. Checks below are not
hardware or flight certification.

## Scope and compatibility

The user selected MATEKH743HD, all configured/initialized I2C buses, and independent
automatic mounting orientation during simultaneous compass calibration. The first
catalog uses existing target I2C MAG descriptors plus LIS3MDL 0x1C. Physical endpoint
addresses are 0x0C, 0x0D, 0x0E, 0x1C, 0x1E and 0x2C on I2C1/I2C2: twelve stable
slots, each identified by bus/address/driver type. Aliases at one bus/address are
alternative drivers, never multiple devices. The catalog does not promise new
MLX90393 SKU addresses not supported by the target descriptors. SPI-internal
MPU9250, MSP and fake streams are outside this I2C discovery path.

Only initialized buses are searched; enum presence alone is not initialization.
Each source owns persistent descriptor, bus and driver state. Occupied endpoints
of other subsystems are not reprobed. Inventory is built at startup; unplugged
sources remain identifiable and are retried. Hot-add of previously unknown devices
is not promised. Configuration/calibration keys include physical identity, never
compact discovery order. Changing bus, address or detected model invalidates the
old coefficients. An identical replacement at the same endpoint has no unique
hardware serial ID in this protocol and requires manual recalibration.

`multi_mag_enabled` defaults OFF in new PG1050. Legacy PG_COMPASS_CONFIG and
PG_MULTI_SENSOR_CONFIG are unchanged. OFF retains the existing single compass
path. First enabling multi requires a fresh simultaneous calibration; legacy
coefficients are not guessed onto newly discovered devices.

## Acquisition and calibration

Drivers expose READY/NOT_READY/ERROR alongside the existing bool API. NOT_READY
never refreshes sample age or erases an otherwise fresh measurement. ERROR removes
that source's usable measurement, with recovery initialization independent of the
others. Runtime driver reinitialization is permitted only while disarmed because
existing initialization routines contain blocking delays. Transient read failures
continue polling the existing driver and successful reads cancel a pending reset.
VCM has no DRDY and uses a documented ODR/mode guard; it cannot prove that
the chip is not internally frozen. This is an explicit hardware limitation.

One calibration request starts all devices even if one is currently unreadable.
Offset, axis scale, samples and orientation buffers are independent. Results are
staged and validated; solver failure, insufficient coverage, zero/invalid gain or
ambiguous orientation must not overwrite previous valid coefficients. Completion
and EEPROM save occur once for the group; one source failure does not abort others.
Orientation searches are finalized at most one source per compass task to avoid a
large synchronous burst. Legacy auto-orientation's original16 candidates remain;
multi uses48 discrete upright/inverted/side mount candidates with45degree yaw
steps. Extended detection additionally requires axis coverage of at least512
reference units and earth-vector RMS chord residual below0.17431 (~10degrees).
Unsupported oblique mounts with larger residual and ambiguous results are rejected;
a small remaining installation error within that tolerance is possible. Actual mounting/heading needs bench confirmation.

Calibrated vectors use the existing reference magnitude1024, not a physical-gauss
claim. Each source's mounting transform precedes fusion. Tailsitter frame changes
must be applied exactly once relative to the frame at calibration time. Board
mounting/alignment changes require recalibration.

## Trust and fallback policy

MAG uses its own policy so existing IMU/BARO semantics do not change. Every strictly
positive weight participates; zero weights do not participate in ordinary fusion.
Freshness, successful reads, finite nonzero vectors and valid calibration/orientation
are prerequisites. Stale/future/NACK/nonfinite/overflow data cannot be fallback.

Initial engineering defaults (not bench-tuned): maximum age250ms; normal direction
agreement20degrees; hard disagreement45degrees confirmed100ms; calibrated magnitude
hard band35–250percent of1024, good recovery band50–180percent. Soft disagreement
reduces weights; confirmed outliers have exactly zero trust. Recovery requires
one second of good new samples, then a5second time constant. Repeated cached
samples cannot restore confidence. A read outage invalidates the old earth-frame
reference so it cannot permanently prevent recovery after the aircraft is moved.

With3+ sources use median normalized direction; with2 use pair agreement and,
where available, earth-frame history using current attitude. With1 use available
earth-frame history and magnitude/freshness. Gyro-unavailable temporal comparisons
must not reject common rapid movement merely from body-frame change. Magnetic
errors shared by all devices cannot be reliably detected by consensus alone.

Maintain an actual reference ID before total loss of positive trust. If all
weights are zero, use that reference's current usable sample directly without
inflating its trust. If it is unavailable, choose the smallest stable available
ID. No usable device means zero aggregate and normal unhealthy compass behavior.
UI/OSD must distinguish this fallback from healthy trust and from frozen data.

## Protocol and presentation

The existing inventory v1 prefix17bytes is retained; capability flags bit3=supported,
bit4=enabled. Command0x20A4 is a separate paged state protocol; old211byte IMU/BARO
state is unchanged. Requests: cursorU8, expected generationU16LE (0 wildcard).
Reply header24bytes: version1, flags, total, cursor, count<=4, next(255end),
generationU16, captureUsU32, referenceUIDU32, fallbackUIDU32, calibrationRunIdU32.

Each48byte record: UIDU32, slot/type/bus/addressU8, flagsU16, confidenceU16(0..1000),
ageUsU32, raw3I16, aligned calibrated3I32, calibrationStatusU8,
orientationQualityU8, angles3I16decidegrees, sampleCountU16, failureU8, reserved3U8.
Positive trust is quantized upward to at least1; positive-trust flag is explicit.
Flags: detected1,readable2,fresh4,calibrated8,calibrating16,contributing32,
fallback64,positive128,orientationValid256,calibrationFailed512,readError1024.
Inventory includes boot-detected sources even when temporarily missing. Pages are
combined only within the same generation; stale tab replies are discarded.

Configurator adds a localized mode switch and per-source cards under the existing
MAG checkbox, preserving aggregate plots. Cards show identity/endpoint, XYZ, trust,
role, calibration and automatic orientation outcome. Multi calibration follows
actual per-source/run status; old firmware retains the old flow. The global
single-MAG alignment wizard must not pretend to configure independently mounted
multi sources.

## OSD and Blackbox

MAG zero-trust transitions generate one identified LOST event; restoration is a
separate transition. A persistent MAG fallback label remains distinguishable after
the one-off event. Existing IMU/BARO notification thresholds remain unchanged.

Blackbox retains the existing aggregate compass I/P fields unchanged. Individual
sources share ONE composite S frame appended after the existing slow fields, with
static ID/type/bus/address mappings in headers. All source values coexist in that
frame; there is no round-robin overwrite in the standard decoder. A snapshot is
polled at most100Hz, and a changed quantized snapshot forces S before the next
recorded I/P. CSV rows retain the latest complete S snapshot between updates.

Individual encoding uses calibrated XYZ/16, clipped to[-127,127], plus packed
trust8/flags6 including a clipped indicator, in TAG8_4S16 groups. Each source adds
at most6bytes per S frame, up to72bytes for twelve. Raw axes remain available in
telemetry rather than doubling logging bandwidth. Positive trust rounds upward;
zero trust remains exactly zero. Without clipping, component quantization error
is at most8 reference units; this is not a universal heading-error bound.

The standard blackbox-tools decoder has a256byte frame limit. The production
writer fixture measures143bytes for zero native HD state plus12 MAGs and280bytes
for a deliberately bounded stress combination of all HD slow metadata and12 MAGs.
Existing slow fields alone in that stress profile use208bytes. Seven
MAGs fit that measured stress profile (250bytes); this is not a proof of a
universal bound over every legacy field value. Eight or more plus all
large HD slow fields can exceed the old decoder limit. No source is silently
dropped to hide this. Such profiles require a decoder with a larger frame limit
or a reduced logging configuration; the software-only report must retain this
compatibility limitation. Existing I/P sizes do not grow from multi-MAG.

## Required verification record

Host acquisition/driver tests, legacy regression, orientation-context isolation,
fusion positive/zero weights, calibration isolation/persistence, malformed paging,
OSD transitions/fallback, and real Blackbox encode/decode are required. Frontend
requires full tests and browser checks of cards/calibration and old firmware.
Record exact commands/results separately after execution. Product builds and
Flash/RAM/packaged artifact checks follow implementation/QA; publishing/flashing
requires an explicit user request. No physical calibration, failure injection or
flight test is implied by passing software checks.

## Software acceptance results (2026-11-22)

Independent review of the acquisition/fusion/calibration path and a separate
review of MSP/OSD/Blackbox integration found no further blocking source issue.
Hardware acceptance remains outstanding.

The following host executables, built with `cmake --build build/unit --target
<TARGETS> -j4`, passed 125 tests in total: compass_multi_bus(8),
compass_orientation(10), barometer_compass_bus(14), multi_mag_core(10),
mag_fusion(7), multi_mag_log(4), osd_multi_mag(3), blackbox_encoding(5),
multi_acc(15), multi_baro(18), multi_gyro(12), multi_sensor_log(6),
multi_sensor_msp(4), multi_sensor_telemetry(9), each with `_unittest` suffix.
Executables were run directly; this build's CTest discovery was not used.

Acquisition/core/fusion/orientation/log/OSD/encoding executables were additionally
recompiled and run with `-fsanitize=address,undefined -fno-omit-frame-pointer`
and `UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1`:47 tests passed.
ARM syntax checks with GCC13.2.1, `-Wall -Wextra -Werror`, passed for the
changed integration and generated settings on MATEKH743HD and MATEKH743.

Reproducible full writer verification:

```sh
cmake --build build/unit --target multi_mag_log_unittest multi_sensor_log_unittest compass_orientation_unittest bitarray_unittest -j4
python3 src/test/support/check_multi_mag_fullwriter.py --decoder /path/to/blackbox_decode
```

The existing INAV blackbox_decode9.0.0 validation executable decoded all12 source
sets into five main CSV rows with identity headers, changed XYZ and a zero-trust
fallback transition. See `src/test/support/multi_mag_blackbox_testing.md` for the
143/208/250/280byte production-frame fixtures and explicit compatibility limits.

Firmware acceptance build command (prepend the installed GCC13.2.1 bin to PATH):

```sh
cmake --build build/firmware --target MATEKH743HD MATEKH743 -j4
```

Both targets linked successfully. The HD build includes multi-MAG; ordinary
MATEKH743 verifies compilation with that feature disabled. Neither was flashed.
Configurator's full suite, real browser DOM checks, C-serializer fixture checks
and macOS packaging are recorded in its development note and delivery report.

Bench work still required: physical simultaneous calibration and orientations,
actual EEPROM reboot persistence, I2C timing/fault injection, magnetic interference,
fallback under real motion and logged-field throughput on the installed storage.
The software tests mock transport and therefore do not prove these properties.

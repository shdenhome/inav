# MATEKH743HD: three IMUs and two barometers with adaptive confidence

Implementation status: code and software tests, **not hardware acceptance**.
No flashable firmware or Configurator build has been produced for this change.
Full target linking, memory usage and bench measurements remain pending an explicit
build request. Scope is the existing onboard IMUs and MS5611/SPL06, not new sensor
hardware IDs or third-party firmware libraries.

## Confirmed hardware

Use **MATEKH743HD**. Target selection was confirmed on the controller using the
existing custom INAV 10.0.0-rc2 build; this does not validate the new fusion code.
The standard MATEKH743 variant has a different MAX7456 path and is not the selected
hardware target for this feature.

| Physical tag | IMU | Bus | Existing target CS | Sensor alignment |
| --- | --- | --- | --- | --- |
| 0 | ICM-42688-P | SPI1 | PC15 | CW0_DEG_FLIP |
| 1 | ICM-20602 | SPI4 | PE11 | CW0_DEG_FLIP |
| 2 | ICM-42688-P | SPI4 | PC13 | CW90_DEG_FLIP |

These are existing target assignments. The supplied sensor-side schematics show
SPI1_IMU1_CS, SPI4_IMU2_CS and SPI4_IMU3_CS, but do not independently show their MCU
GPIO connections. No pins or orientation constants were changed. The two SPI4 IMUs share SPI4_INT.
Multi mode therefore sets a per-device polling-only flag: ICM426xx DRDY mapping is
disabled before gyro wake and not re-enabled; MPU6500-family INT_ENABLE is explicitly
cleared before wake. A failure to perform this step marks initialization failed and
excludes that device while allowing another IMU to become primary. Legacy mode
retains the previous interrupt configuration. Electrical behavior still needs bench
verification; code/register tests do not validate the physical shared net.

Both barometers use I2C2: **MS5611 at 0x77** and **SPL06 at 0x76**, according to the
existing descriptors and supplied bus schematics. Physical jumper population must
permit both addresses. The user confirmed this pair is populated; actual successful
concurrent detection still needs to be verified on hardware. LIS3MDL remains on
I2C1, with its existing driver and address handling unchanged.

## Configuration and compatibility

Only MATEKH743HD defines USE_MULTI_SENSOR. Existing single-sensor operation remains
the default; new Parameter Group PG_MULTI_SENSOR_CONFIG (1049, version 0) contains
all additional configuration and accelerometer calibration. Existing PG layouts,
versions, enum values and defaults were not changed. The earlier proposed ICP20100
enum extension is no longer needed.

| CLI setting | Default | Meaning |
| --- | --- | --- |
| multi_imu_enabled | OFF | Concurrent IMU acquisition and confidence-weighted gyro/accel output |
| multi_baro_enabled | OFF | Concurrent MS5611/SPL06 acquisition and weighted pressure |
| multi_gyro_deviation | 100 | Gyro disagreement threshold, deg/s |
| multi_acc_deviation | 300 | Acceleration disagreement threshold, milli-g |
| multi_baro_deviation | 100 | Pressure disagreement/innovation threshold, Pa |
| multi_sensor_recovery_ms | 5000 | Confidence recovery time constant, ms |
| multi_imu_max_age_ms | 10 | Maximum usable sample age, ms |
| multi_baro_max_age_ms | 200 | Maximum usable sample age, ms |

The two enable flags are latched during initialization: changing a setting in CLI
cannot switch acquisition topology live. Save and reboot to enable/disable a mode.
Thresholds are preliminary, configurable engineering defaults, not validated flight
limits. Maximum age must exceed the physical sensor/conversion period; an overly
short age intentionally makes that device unavailable. The Configurator has no new custom GUI controls; CLI is the current interface.

Example after explicitly requested firmware building and flashing:

```text
set multi_imu_enabled = ON
set multi_baro_enabled = ON
save
```

`gyro_to_use` selects the preferred primary descriptor. Other available tags are
opened independently. If it is absent at initialization, the multi mode tries the
other tags. Physical calibration follows descriptor tags, not packed runtime slots.
Single-sensor `baro_hardware` selection is bypassed while multi_baro_enabled is on.
Do not infer two healthy barometers just from the legacy single BARO label in status.

## IMU processing

Each gyro retains its own bus device, sampling interval, zero calibration, scale and
alignment. All detected gyros calibrate their own zero at startup in multi mode;
the legacy saved gyro-zero values are neither reused nor overwritten by this path.

Reads are scheduled at each device's own interval, with the consumer cadence set
to the fastest initialized member so a slow preferred primary cannot throttle peers. Each device has one contribution
from its latest sufficiently fresh sample, irrespective of the consumer polling
rate. The path is raw sample -> individual zero -> sensor/board alignment -> deg/s
-> confidence-weighted mean -> existing common gyro filters and attitude/PID path.
The existing secondary Blackbox channel, if enabled, receives the calibrated second
runtime slot; the aggregate is the normal gyro channel. Third-device values and all
confidence values are available through CLI status, not new Blackbox/MSP fields.

The ICM-20602 accelerometer consumes the gyro driver's shared register cache. Its
sample retains the gyro transfer timestamp; repeated reads of that cache are not
relabelled as fresh and do not count as new calibration/trust observations.

Accelerometers have separate device contexts, six-face calibration and saved offsets/
gains by physical tag. Raw readings are normalized to 4096 units/g and aligned to
body axes for calibration. The output path is individual body-axis offset/gain -> g
-> weighted mean -> one common board alignment -> existing acceleration filters.
Legacy primary accelerometer coefficients must not be applied to the aggregate.

Use the existing six-position calibration UI, beginning top-up. Every detected
accelerometer must provide fresh readings to start a calibration face. Coefficients
are written only after all six faces succeed for all detected accelerometers.
Incomplete calibration prevents those accelerometer measurements from contributing.
A detected but unreadable IMU can prevent calibration completion; no automatic
arming/failsafe policy to bypass calibration was introduced.

CLI dump includes multi_acc_calibrated_mask plus multi_acc_zero_first_x through
multi_acc_gain_third_z. First/second/third denote physical tags 0/1/2, not primary/
secondary runtime slots. Recalibrate after changing the populated IMUs or mounting.

## Barometer processing

MS5611 and SPL06 retain separate conversion state machines and deadlines. Each
starts temperature, waits its own temperature delay, reads temperature, starts
pressure, waits its pressure delay, reads and compensates pressure, then repeats.
A failed read/start invalidates that sensor and restarts its temperature cycle;
its conversion wait does not block the other device or feed uninitialized/stale
conversion data. The underlying I2C driver is still synchronous; a physically stuck
shared bus can affect both sensors and needs bench fault testing.

During startup, two stable, fresh barometers first learn their pressure difference
for a CALIBRATING_BARO_TIME_MS window. Half this difference is added to MS5611 and
subtracted from SPL06, preserving the unweighted pressure mean while aligning their
ground datum. The common ground calibration then restarts in that datum. This takes
approximately two calibration windows when both devices are present, not just one.
An absent device needs no pair alignment; a present but unreadable device prevents
pair calibration completing. A difference exceeding 10 * multi_baro_deviation is
not silently calibrated away: pair calibration stays pending. Offsets are retained
through later communication failures and are not recalibrated to an airborne height.

Aligned pressure samples in Pa are confidence-weighted, then sent to the existing
single altitude conversion and ground calibration. This prevents a static factory
offset from causing an artificial altitude step when a contributor drops out. It is
**pressure averaging in a common datum**, not averaging independently computed altitudes. Navigation receives updates only when
there is a new usable pressure measurement, not on every intermediate conversion
state. With no usable measurements, no new navigation sample is published.

Temperature is weighted for display. Both drivers already perform their own factory
pressure/temperature compensation. The legacy single-sensor baro_temp_correction
fit is intentionally not applied to a blend of two different barometers; it remains
unchanged in single-barometer mode.

MS56xx now propagates ADC read, reset-write and PROM-read failures. Previously ADC
NACK could consume an uninitialized buffer and report success. Valid conversions retain the existing compensation algorithm and outputs. The
ICM426xx/MPU6500 identity probes also now check read acknowledgements instead of
switching on an uninitialized byte after NACK. These are intentional error-behavior
corrections; sensor identity constants, ranges, rates and normal legacy configuration
remain unchanged. The legacy barometer loop itself still
ignores operation return values, while the multi loop checks every conversion stage.

## Confidence algorithm and limitations

Each gyro, each accelerometer and each barometer has independent history and
confidence. Angular rates, accelerations and pressure never share one confidence
state or raw threshold.

- Three available measurements use the component-wise median as the disagreement
  reference, so a coherent common motion is not rejected as a temporal jump.
- With two measurements, disagreement is evaluated using each source's innovation
  relative to its own accepted history and smoothed trend. With one measurement,
  only temporal consistency is available. Trend extrapolation is time-bounded.
- Normalized score <= 1 targets confidence 1; larger scores target 1/score^2.
  A decrease is immediate. Recovery toward 1 follows a first-order time constant
  and is updated only on a new sample from that sensor. Suspect spikes do not
  replace accepted history.
- Confidence below 0.1 is quarantined and contributes zero effective weight until
  sustained recovery. Read failure, NaN/Inf, stale, future or out-of-order data
  immediately contributes zero. Repeated reads of a cached sample cannot restore
  trust. Timestamp-wrap behavior is tested.
- The aggregate is sum(weight * sample) / sum(weight) over eligible sources.
  If none qualify, health becomes false and the last output is retained as history;
  it is **not** published as a fresh healthy measurement. No synthetic zero sample
  is substituted for a failed read.

Two disagreeing barometers cannot uniquely identify a bad source without additional
information. Slow drift, a frozen sensor during steady conditions, common-mode faults
and two faulty IMUs agreeing against a good third can evade this algorithm. This is
confidence-weighted estimation, not guaranteed fault isolation. After a prolonged acquisition gap, expired temporal history can be reacquired
without restoring full confidence immediately; a returning source that still disagrees
with a healthy peer stays quarantined. Different sensor
latencies are bounded by maximum age, not corrected by time interpolation.

No new motor-disarm, in-flight failsafe or navigation-source-switching policy is
introduced. Loss of all usable gyro/acceleration inputs is reported unhealthy; a
continued safe flight under total IMU failure is not established.

## Diagnostics

`status` appends runtime slot -> physical tag, each gyro/accel confidence (0..1000),
validity/age, individual calibrated vectors, barometer pressures/confidence and
aggregate contributor counts/health and learned barometer datum offsets. Legacy MSP sensor IDs remain unchanged.
A missing device must be distinguished from a low-confidence or stale device using
these fields; the existing single sensor-type label is insufficient.

## Software verification

Focused tests execute production fusion, gyro/acceleration processing, barometer
state machines, calibration mathematics and Parameter Group store/load code with
mocked hardware. They cover physical scale/orientation conversion, independent zeros,
six-face calibration, individual persistence by tag, asynchronous rates/conversions,
spikes, recovery, missing sensors, NACK, invalid/future/stale data, clock wrap,
all-sources-rejected health and disabled-mode defaults.

Result: 87 focused/baseline tests pass. Sanitized runs of 73 cases across seven
executables pass with AddressSanitizer and UndefinedBehaviorSanitizer. Existing
ICM426xx/BMP388/BMP390/LIS3MDL regression tests remain included. New MS5611 tests
preserve the datasheet compensation vector (100009 Pa, 20.07 C) and verify failures.

ARM GCC 13.2.1 syntax/type checks with -Wall -Wextra -Werror pass for changed modules
and CLI under both MATEKH743HD (feature enabled) and MATEKH743 (feature absent).
Settings generation succeeds for both targets. These checks do not replace target
linking or a Flash/RAM report.

The full host `check` build encounters a pre-existing MAVLink test error at
src/test/unit/mavlink_unittest.cc:208 (C++ narrowing uint16_t -> int16_t). The same
error was reproduced against the unchanged baseline source. No tests were removed,
no assertions weakened and no unrelated MAVLink source was edited. The complete CTest run reports 618 passed out of 619 entries; the remaining entry
is the unbuilt MAVLink executable, not a new sensor test failure. The command still
returns failure and is not represented as a fully green suite.

The 27 new Settings.md entries were generated using the standard documentation
tool. An existing manually customized gimbal_mode_channel paragraph was preserved:
regenerating the entire document would also rewrite that unrelated baseline entry.
No unrelated gimbal behavior or descriptions were changed.

## Reproduce focused host checks

```sh
cmake -S . -B build/unit -DTOOLCHAIN=none
cmake --build build/unit --target sensor_fusion_unittest multi_gyro_unittest multi_acc_unittest multi_baro_unittest accgyro_icm42605_unittest accgyro_mpu6500_bus_unittest barometer_ms56xx_bus_unittest barometer_compass_bus_unittest -j 4
ctest --test-dir build/unit/src/test --output-on-failure -R 'SensorFusionTest|MultiGyroTest|MultiAccTest|MultiBaroTest|Icm426xxBusTest|Mpu6500BusTest|Ms56xxBusTest|ExistingSensorBusTest'
```

The two IMU integration fixtures deliberately focus on acquisition/calibration/fusion;
optional CMSIS FFT/Kalman/adaptive-filter execution is outside those host fixtures.
Real-target syntax checks include the actual enabled code paths, but target runtime
filter timing and latency still need the build/bench acceptance below.

## Remaining hardware/build acceptance

- Explicitly requested complete target build, linker warnings and Flash/RAM usage.
- Confirm all three physical IMUs and both barometers are actually detected.
- Six-face calibration, reboot/storage, individual axis directions and common board
  alignment on the real controller; inspect all confidence/status fields.
- Compare individual and aggregate noise, latency, vibration and CPU/task load.
- Bench verification of shared SPI4_INT electrical levels and fault injection/recovery,
  including either/both barometers, any/all IMUs,
  disconnected buses and data freezes. Software mocks do not prove these cases.
- Check altitude continuity and estimator behavior as contributor weights change.
- Long-duration operation and thermal testing. **-40 to +85 C is not validated.**
- No hardware trials or flight-readiness claim is made by this change.

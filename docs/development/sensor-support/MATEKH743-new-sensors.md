# MATEKH743: ICM-42688 and ICP20100 support work

Status: investigation and baseline tests only. ICM42688P is already supported by the
existing driver; ICP20100 support and the requested hardware configuration are
**not implemented or hardware-verified**.
Do not flash an existing MATEKH743 build expecting these new sensors to work.

## Scope and agreed interface change

Requested (corrected by the user): two ICM-42688 IMUs, an ICP20100 barometer, and verification of the existing
BMP388 and LIS3MDL support in the custom INAV 10 firmware.

The user confirmed MATEKH743 as the target and approved appending ICP20100 to the
barometer hardware list without changing existing numeric IDs or defaults.
The current last value is BARO_CRSF = 13. ICP20100 must be appended as 14 when its
working driver is registered; do not expose a nonfunctional selection before then.
No parameter-group layout change is necessary for an additional enum value.

Base commit: 054f26577100e6fdc76c8078f5c1c7d0b21fdb7b.
Branch: codex/iim42652-icp20100 (created before the sensor-name correction).

## Existing architecture

- `accgyro_icm42605.c` already handles ICM42688P (WHO_AM_I 0x47), including
  ranges, sample rates and its AAF table. The corrected request no longer requires
  adding an IIM-42652 identity or a driver for that part. Confirm the exact chip
  suffix/revision before claiming compatibility with another variant.
- MATEKH743 already registers the ICM426xx family at tag 0 (SPI1, CS PC15) and
  tag 2 (SPI4, CS PC13). These descriptors can discover ICM42688P with the existing
  driver if the physical wiring and orientation match. This does not establish
  which two chips are installed on the user's board.
- `gyro_to_use` selects the active hardware descriptor tag. The accelerometer
  follows that IMU. `gyro_secondary_enabled` permits acquisition of another gyro
  for logging; it is not automatic failover or simultaneous attitude fusion.
- The ICM driver currently stores its variant in one static variable. Test any
  future changes with both identical and mixed devices and both initialization
  orders. Do not introduce a new failover mechanism as part of this work.
- The barometer subsystem has a single active `baroDev_t`. Two physical barometers
  can coexist on a bus, but hardware selection/auto-detection chooses one for
  altitude estimation. This work must not claim concurrent barometer fusion.
- Existing barometer auto-detection priority and all previous `baro_hardware`
  values/defaults must remain unchanged when ICP20100 is added.

## Target configuration and missing wiring

These are existing source definitions, not a proposed connection diagram:

| Interface | Existing MATEKH743 definitions |
| --- | --- |
| SPI1 | SCK PA5, MISO PA6, MOSI PD7; IMU tag 0, CS PC15 |
| SPI4 | SCK PE12, MISO PE13, MOSI PE14; MPU6500 family tag 1, CS PE11; ICM426xx tag 2, CS PC13 |
| SPI3 external | SCK PB3, MISO PB4, MOSI PB5; RM3100 already uses CS PE2 |
| I2C1 | SCL PB6, SDA PB7; default magnetometer bus |
| I2C2 | SCL PB10, SDA PB11; default barometer bus, also used by battery monitor |

The target enables BMP280, MS5611, DPS310 and SPL06, but not BMP388.
LIS3MDL is included through USE_MAG_ALL. Its common descriptor currently uses
address 0x1e; the alternative physical address is not automatically probed.

Still required:

1. Exact board model/revision and whether the new IMUs replace onboard chips or
   are external modules.
2. SPI number, CS pin and mounting orientation of each ICM-42688; INT pin if wired.
3. Bus and physical address of ICP20100, BMP388 and LIS3MDL.
4. Accessible manufacturer documentation for ICP20100 and the exact ICM-42688 variant, including
   revision-specific initialization/OTP requirements.

The user supplied the signal label `spi1_int`. Its physical GPIO, device association
and CS connection remain unspecified; an INT label is not a chip-select assignment.

No target pins, addresses or orientation were guessed. The target shared with
MATEKH743HD has not been modified.

## Baseline software tests

Added real-driver tests with emulated bus transactions:

- `accgyro_icm42605_unittest`: 5 tests covering the three existing identities,
  ranges/AAF configuration, all five existing sample rates, signed big-endian
  data, independent reads and alignment metadata for two identical ICM42688P
  devices, read-error output retention and recovery, absent devices and unpaired
  accelerometer rejection. The actual rate-selection function is linked.
- `barometer_compass_bus_unittest`: 14 tests covering BMP388/BMP390 identification,
  I2C and SPI dummy bytes, a synthetic compensation vector, forced conversion,
  measurement-start errors, LIS3MDL configuration, signed little-endian axes,
  ready status, selected descriptor tag and transport-error recovery.

Result: **19/19 pass on the unchanged production drivers** with host Clang on macOS.
Tests confirm only the exercised software paths, not sensor accuracy, hardware
wiring, timing, noise, temperature operation or support for the new parts.

The synthetic BMP388 compensation vector is an exactly soluble arithmetic fixture;
it is not a captured calibration/readout from a real sensor.

### Existing error-handling limitations reproduced

Three passing characterization tests deliberately record current shortcomings;
they must not be interpreted as successful error handling:

- BMP388 measurement read NACK returns success and reuses the previous sample.
- BMP388 calibration read NACK does not fail detection.
- LIS3MDL configuration write NACK does not fail initialization.

Production behavior is unchanged. Fixing these limitations is a separate, explicit
behavior change under the task's compatibility constraint and needs agreement.
The existing ICM driver also ignores WHO_AM_I read acknowledgement and several
initialization transactions; new hardware must not be considered verified on this basis.

The host build of the existing ICM source emits two Clang warnings for the existing
logical-OR expression in ICM42605_UI_DRDY_INT_CLEAR_ON_SBR. No warning suppression
or production change was added. The new barometer/compass test target builds without
warnings. A target firmware warning/memory comparison remains pending implementation.

### Reproduce

From the repository root, with the project's GoogleTest dependency available:

```sh
cmake -S . -B build/unit -DTOOLCHAIN=none
cmake --build build/unit --target accgyro_icm42605_unittest barometer_compass_bus_unittest -j 4
ctest --test-dir build/unit/src/test --output-on-failure -R 'Icm426xxBusTest|ExistingSensorBusTest'
```

For this run, the existing local GoogleTest checkout was reused because external
HTTPS requests failed. No external dependency or production source was changed.

## Pending implementation and acceptance

- Verify both new devices against manufacturer register maps and initialization
  requirements; implement guarded drivers and bus registrations.
- Bind them to confirmed target wiring, retaining all other target configurations.
- Add new-device tests: identity/NACK, initialization, signed samples, physical
  units, stale/invalid measurements, bounded waits and recovery. Exercise both IMUs
  and both barometer selections; compare existing-driver baseline again.
- Build MATEKH743 and affected configurations; record warnings, Flash and RAM.
- Review source scope and prepare a draft PR against the user's custom INAV branch.
- Run on actual hardware: reboot/persistence, calibration, orientation, UART/MSP/CLI,
  scheduler timing, altitude behavior, vibration and prolonged operation.
- Temperature testing over -40 to +85 C has **not** been performed or established.

No firmware artifact, completed-support claim or ready-to-merge PR is produced at
this baseline-only stage. Hardware acceptance remains open.

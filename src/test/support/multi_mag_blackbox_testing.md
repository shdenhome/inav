# Multi-MAG Blackbox host QA

Run from the firmware repository after configuring the host unit build:

```sh
cmake --build build/unit --target multi_mag_log_unittest osd_multi_mag_unittest multi_sensor_log_unittest compass_orientation_unittest bitarray_unittest -j4
build/unit/src/test/unit/multi_mag_log_unittest
build/unit/src/test/unit/osd_multi_mag_unittest
python3 src/test/support/check_multi_mag_fullwriter.py --decoder /path/to/blackbox_decode
```

`--build` selects another existing host unit build. The script reads its
`compile_commands.json`, uses its compiler/defines/include paths and links its
production encoder objects. It does not download tools or build firmware.
The fixture includes the real `blackbox.c`; only byte transport and unused
motor/servo queries are stubbed. `writeSlowFrame()` and the S field definitions
are production code. Minimal main I/P fields isolate the S snapshot contract.
The optional HD S field superset includes temperature, ESC, terrain, DroneCAN,
MSP override and existing multi-IMU/BARO telemetry. Identity headers are
synthetic fixture IDs, not claims about attached hardware.

The generated `build/unit/qa-multi-mag/fullwriter.bbl` contains all twelve
sources in every composite S snapshot. Five main rows verify all twelve sets
of XYZ/quality, new XYZ values and a transition to zero-trust fallback. Static
ID/type/bus/address header mappings are checked. Decoding with the existing
INAV blackbox_decode 9.0.0 validation binary passed without corrupt frames.
The CSV retains the latest complete S snapshot for each main row; it never
relies on multiple per-device S frames surviving between main frames.

Measured **complete encoded S frame sizes, including marker**:

| Fixture profile | Bytes |
| --- | ---: |
| Zero-valued existing fields + twelve maximum-size MAG tuples | 143 |
| Typed bounded optional-field stress, existing fields alone | 208 |
| Same stress + seven maximum-size MAG tuples | 250 |
| Same stress + twelve maximum-size MAG tuples | 280 |

These stress values are an explicit reproducible input set, not a proof that
208 bytes bounds every possible legacy state. Each MAG tuple adds at most six
bytes. A decoder with the existing 256-byte frame cap cannot reliably decode
the 280-byte stress S frame. No sources or axes are silently dropped to fit
that cap. The 143-byte all-source fixture passes the existing decoder. This
limitation must remain visible in acceptance/build notes; passing this fixture
is not universal decoder compatibility or hardware acceptance.

XYZ uses calibrated/aligned reference units divided by 16. Rounding introduces
at most 8 units per axis (vector error <= 13.857 reference units). At vector
norm 1024 the direction error bound is approximately 0.78 degrees; heading
error also depends on horizontal field and is unbounded near a vertical field.
Clipped values carry a flag and have no such direction-error guarantee.
Quality bits 0..7 hold ceil(confidence*255), preserving exact-zero versus
positive trust. Bits 8..13 hold readable/fresh, contributing, fallback,
calibrated, calibrating and clipped flags. All source tuples are absolute S
values. Snapshot polling is capped at 100 Hz and unchanged quantized snapshots
are not emitted solely for MAG; normal native S scheduling remains active.

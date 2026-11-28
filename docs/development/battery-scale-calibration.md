# Battery scale calibration support

The Configurator calibration workflow needs four individually writable RAM
settings. Battery1 already has `vbat_scale` and `current_meter_scale`.
Battery2 adds `battery2_voltage_scale` and `battery2_current_scale`, both uint16
with range1..65535. They bind existing PG_BATTERY2_CONFIG members; the structure
and its version1 remain unchanged. Factory defaults remain1000/2 (I2C).

The ordinary common-setting MSP setter performs point writes without EEPROM.
Its uint16 path now checks the declared lower bound as well as the upper bound.
This prevents zero battery2 scales through MSP; calibration does not use the
multi-field setters to transmit unrelated unsaved fields.

The existing battery2 multi-field setter still supports15/20/21-byte payloads.
A21-byte packet now consumes the same five source/enable/CAN fields as20bytes,
then the address byte. Decoding/validation is staged before assigning the PG;
invalid or truncated packets cannot partially change RAM.

Battery2 initialization no longer guesses legacy units from numeric values.
The former1000/2100 voltage and2/>=100 current heuristics could also match valid
new calibration values and overwrite offsets. Persisted coefficients are now
authoritative. Explicit source changes in Configurator retain their existing
source-specific default seeding. No automatic PG reset is introduced.

Firmware transformations are direct for ADC voltage1; inverse for ADC current1;
direct plus additive offset for ADC/I2C voltage2; inverse after input offset for
ADC/I2C current2. CAN and first-battery INA226 bypass these ADC scale members.
MSP ANALOG voltage follows the selected Raw/Sag compensated mode, so calibration
must use Raw. See Configurator docs/development/battery-calibration.md for the
actual formulas, UI validation and ACK/readback state machine.

Verification (no product image build): battery2_config_msp_unittest4 tests cover
packet versions, rejected/truncated data and uint16 bounds; battery2_init_unittest
covers24 source/scale combinations with nonzero offsets and repeated actual init.
ARM syntax checks of fc_msp.c and battery2.c with -Werror passed on MATEKH743HD.
Metadata audit found no uint16 numeric defaults below their declared minima in
194 host settings and228 generated MATEKH743HD settings. Instrument comparison
and real EEPROM save/reboot remain hardware acceptance work.

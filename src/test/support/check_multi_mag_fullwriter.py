#!/usr/bin/env python3
"""Exercise production writeSlowFrame with the host-unit compile environment.

First build multi_mag_log_unittest, multi_sensor_log_unittest,
compass_orientation_unittest and bitarray_unittest in a UNIT_TESTS build.
Optionally pass --decoder /path/to/blackbox_decode (no downloads performed).
"""
import argparse
import csv
import json
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--build', type=Path, default=ROOT / 'build/unit')
p.add_argument('--decoder', type=Path)
args = p.parse_args()
build = args.build.resolve()
entries = json.loads((build / 'compile_commands.json').read_text())
e = next(e for e in entries if 'multi_mag_log_unittest.dir' in e['command'] and e['file'].endswith('multi_mag_log.c'))
command = shlex.split(e['command'])
out = build / 'qa-multi-mag'
out.mkdir(exist_ok=True)
command[command.index('-o') + 1] = str(out / 'fullwriter.o')
command[-1] = str(Path(__file__).with_name('multi_mag_fullwriter.c'))
# HD optional S fields: deliberately enable the complete relevant superset.
features = ['USE_MULTI_SENSOR', 'USE_DUAL_GYRO', 'USE_SDCARD',
            'USE_TEMPERATURE_SENSOR', 'USE_ESC_SENSOR', 'USE_TERRAIN',
            'USE_DRONECAN', 'USE_RX_MSP', 'USE_MSP_RC_OVERRIDE',
            'USE_PITOT', 'USE_RANGEFINDER']
command[1:1] = ['-include', 'stdio.h'] + ['-D' + f for f in features]
subprocess.run(command, cwd=e['directory'], check=True)
objects = []
for target, source in [
    ('multi_mag_log_unittest', 'blackbox/multi_mag_log.c'),
    ('multi_mag_log_unittest', 'blackbox/blackbox_encoding.c'),
    ('multi_mag_log_unittest', 'common/encoding.c'),
    ('multi_sensor_log_unittest', 'blackbox/multi_sensor_log.c'),
    ('compass_orientation_unittest', 'common/maths.c'),
    ('bitarray_unittest', 'common/bitarray.c'),
]:
    entry = next(e for e in entries if target + '.dir' in e['command'] and e['file'].endswith(source))
    words = shlex.split(entry['command'])
    obj = Path(words[words.index('-o') + 1])
    if not obj.is_absolute():
        obj = Path(entry['directory']) / obj
    if not obj.exists():
        p.error('Build required unit target first: ' + target)
    objects.append(str(obj))
exe = out / 'fullwriter'
strip = '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections'
subprocess.run([command[0], strip, str(out / 'fullwriter.o'), *objects,
                '-lm', '-o', str(exe)], check=True)
log = out / 'fullwriter.bbl'
result = subprocess.check_output([str(exe), str(log)], text=True)
print(result, end='')
assert 'bounded native full HD S=208' in result, result
assert 'bounded full HD S plus12MAG=280' in result, result
assert 'bounded full HD S plus7MAG=250' in result, result
if args.decoder:
    subprocess.run([str(args.decoder.resolve()), str(log)], cwd=out, check=True)
    with log.with_suffix('.01.csv').open() as file:
        reader = csv.DictReader(file, skipinitialspace=True)
        rows = list(reader)
    assert len(rows) == 5, len(rows)
    for frame, row in enumerate(rows):
        for slot in range(12):
            for name, expected in [('X', -100 + frame), ('Y', slot), ('Z', 64),
                                   ('Quality', 3071 if frame < 2 else 12032)]:
                assert int(row[f'magSource{name}[{slot}]']) == expected, (frame, slot, name)
    header = log.read_bytes().split(b'\nS', 1)[0]
    for slot in range(12):
        assert f'H mag_source_{slot}:{100+slot},8,{1+slot//6},{64+slot%6}\n'.encode() in header
    print('PASS: production S writer, five CSV rows, all 12 sources, identity headers, fresh XYZ and zero-trust fallback')
else:
    print('Decoder not supplied: CSV compatibility not checked. Fixture:', log)
print('LIMITATION: optional-field stress S=280 exceeds the legacy decoder 256-byte frame limit; no source data was dropped.')

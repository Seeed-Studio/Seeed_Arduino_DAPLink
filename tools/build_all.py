import os
import shutil
import sys
import subprocess
import time
import glob

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[33mskipped\033[0m"

success_count = 0
fail_count = 0
exit_status = 0
skip_count = 0

build_format = '| {:35} | {:18} | {:6} |'
build_separator = '-' * 60

all_boards = [
    # M0 Boards
    'adafruit:samd:adafruit_feather_m0_express:usbstack=tinyusb',
    'adafruit:samd:adafruit_metro_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_circuitplayground_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_gemma_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_trinket_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_itsybitsy_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_pirkey:usbstack=tinyusb',
    'adafruit:samd:adafruit_hallowing:usbstack=tinyusb',
    'adafruit:samd:adafruit_crickit_m0:usbstack=tinyusb',
    'adafruit:samd:adafruit_blm_badge:usbstack=tinyusb',

    # M4 Boards
    'adafruit:samd:adafruit_metro_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_grandcentral_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_itsybitsy_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_feather_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_trellis_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_pyportal_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_pyportal_m4_titano:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_metro_m4_airliftlite:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_pybadge_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_pygamer_advance_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_pybadge_airlift_m4:speed=120,usbstack=tinyusb',
    #'adafruit:samd:adafruit_monster_m4sk:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_hallowing_m4:speed=120,usbstack=tinyusb',
    'adafruit:samd:adafruit_matrixportal_m4:speed=120,usbstack=tinyusb',

    # nRF Boards
    'adafruit:nrf52:feather52840',
    'adafruit:nrf52:feather52840sense',
    'adafruit:nrf52:cplaynrf52840',
    'adafruit:nrf52:itsybitsy52840',
    'adafruit:nrf52:cluenrf52840'
]

sketch = 'examples/simple_daplink'

if os.path.exists('bin'):
    shutil.rmtree('bin')

os.makedirs('bin')
total_time = time.monotonic()

print(build_separator)
print(build_format.format('Board', '\033[39mResult\033[0m', 'Time'))
print(build_separator)

for variant in all_boards:
    start_time = time.monotonic()

    variant_bare = variant.split(":")[2]
    output_dir = 'build/' + variant_bare;

    # Skip if contains: ".variant.test.skip" or ".all.test.skip"
    # Skip if not contains: ".variant.test.only" for a specific variant
    if os.path.exists(sketch + '/.all.test.skip') or os.path.exists(sketch + '/.' + variant + '.test.skip'):
        success = SKIPPED
    elif glob.glob(sketch+"/.*.test.only") and not os.path.exists(sketch + '/.' + variant + '.test.only'):
        success = SKIPPED
    else:
        build_result = subprocess.run("arduino-cli compile --warnings default --output-dir {} --fqbn {} {}".format(output_dir, variant, sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        if build_result.returncode != 0:
            exit_status = build_result.returncode
            success = FAILED
            fail_count += 1
        else:
            success = SUCCEEDED
            success_count += 1
            family_id = '0xADA52840' if "adafruit:nrf52" in variant else '0x0'
            subprocess.run("python3 tools/uf2conv.py -f {} -c -o bin/daplink-{}-v1.0.0.uf2 {}/simple_daplink.ino.hex".format(family_id, variant_bare, output_dir), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    build_duration = time.monotonic() - start_time

    print(build_format.format(variant_bare, success, "{:5.2f}s".format(build_duration)))

    if success != SKIPPED:
        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))
            if (build_result.stderr):
                print(build_result.stderr.decode("utf-8"))
    else:
        skip_count += 1


# Build Summary
total_time = time.monotonic() - total_time
print(build_separator)
print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(success_count, SUCCEEDED, fail_count, FAILED, skip_count, SKIPPED, total_time))
print(build_separator)

sys.exit(exit_status)

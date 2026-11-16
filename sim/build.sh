#!/bin/sh
# Builds the unmodified production env and copies the ELF to sim/build/canard.elf. Needs PlatformIO and submodules.
set -e
cd "$(dirname "$0")/.."
git submodule update --init --depth 1 src/third_party/CMSIS-DSP src/third_party/closedrocket src/third_party/rocketlib src/third_party/xsens-mti
ENV=${ENV:-release_board_1_ad_09590}
pio run -e "$ENV"
mkdir -p sim/build
cp ".pio/build/$ENV/firmware.elf" sim/build/canard.elf

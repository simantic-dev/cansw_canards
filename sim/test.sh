#!/bin/sh
# Runs the committed canards flight-computer image (fw/firmware.elf, STM32H733) on pyrite for 12 virtual s with the
# sensor, servo, RocketCAN and ground peers and the synthetic flight in flight.py. Pass = the ground station set
# the pad filter and opened the injector, the firmware saw the launch acceleration on the IMU, its controller started
# telemetry, the AK45 servo received a non-zero canard angle command, and there was no HardFault.
cd "$(dirname "$0")" || exit 1
SIM=${SIM:-sim}
O=$(mktemp)
trap 'rm -f "$O"' EXIT
"$SIM" run --scenario scenario.yaml --timeout 12 > "$O" 2>&1
rc=$?
fail=0
chk() { if eval "$2"; then echo "ok   $1"; else echo "FAIL $1"; fail=1; fi; }
chk "sim exited 0" "[ $rc = 0 ]"
chk "ground: pad filter ON" "grep -qF '[ground] pad filter ON' '$O'"
chk "ground: ox injector valve ON" "grep -qF '[ground] ox injector valve ON' '$O'"
chk "IMU configured and sees the launch accel" "grep -qF '[imu] launch-phase sample' '$O'"
chk "controller telemetry started" "grep -qF '[ground] canards ctrl telemetry started' '$O'"
chk "servo got a non-zero canard angle" "grep -E '\[motor\] SET_POS count .* latest -?[1-9][0-9.]* deg' '$O' | grep -q ."
chk "no HardFault" "! grep -qi hardfault '$O'"
exit $fail

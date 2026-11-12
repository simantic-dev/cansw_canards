# Running the canards firmware on pyrite

`pio run -e release_board_2_ad_09590 -t sim` boots the built `firmware.elf` in pyrite, a
firmware simulator for the STM32H733VG, with a set of scripted peers on its buses: the AK45
servo and the sensors on the board, a RocketCAN listener and a ground station on FDCAN3, and a
synthetic flight. No hardware is needed.

Pyrite is a proprietary engine provided to the team by Simantic. Get the `sim` binary from
Simantic and either put it on `PATH` or point `PYRITE_SIM` at it.

```
PYRITE_SIM=/path/to/sim pio run -e release_board_2_ad_09590 -t sim
```

The run takes about 12 virtual seconds, roughly 3 minutes of wall time. The target exits
non-zero if `[motor] SET_POS` never appears on the simulated UART, which means the controller
never sent a canard command to the servo.

## What the run shows

- Peer log lines for the flight: the AK45 feedback stream and its command, the IMU and
  magnetometer configured, the barometer PROM read, the Movella MTData2 stream, the ground
  station's pad filter and injector commands, the first in-flight IMU and barometer samples,
  RocketCAN board status, controller telemetry, and the `SET_POS` canard commands.
- The firmware runs its FreeRTOS tasks on the simulated clock. A HardFault fails the run.

## What is synthetic

- The flight in `flight.py` is 1-D vertical kinematics with no drag, not a recorded flight.
- Every sensor is a scripted peer, so readings and timing are what the peers produce.
  Nothing here is measured against silicon, and timing is not a claim about the hardware.
- The AK45 servo, the ground station, and the RocketCAN listener are stand-ins that
  implement only the frames the firmware exchanges with them.
- The SD card is blank: the logger's file write fails and the run continues.
- The ADXRS649 gyro self-test and the ADXL380 are not served, so their init fails.

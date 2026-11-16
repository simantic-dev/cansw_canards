# Canard Board Firmware

Firmware for 2025-27 Canard Board, performing state estimation and control for the 2nd & 3rd iteration of the canards system. Project documentation can be found in the team Google Drive.
In 2024-25, this project was firmware for Processor Board in the first canards system

## Run it on a simulated board

```sh
git clone --recurse-submodules https://github.com/simantic-dev/cansw_canards && cd cansw_canards
pio run -e release_board_2_ad_09590    # once, separate step: builds the release board 2 firmware
./demo                                 # boots it on the simulated STM32H733 and prints the flight timeline (about 60 s wall)
```

![Flight timeline from the simulated canards run](media/canards-demo.gif)

```mermaid
flowchart LR
  mcu["STM32H733VG<br/>canards firmware (unmodified, release board 2)"]
  motor(["AK45 servo<br/>ak45_peer.py"]) -- "FDCAN1 1 Mbit: SET_POS canard angle" --- mcu
  bus(["RocketCAN listener<br/>rocketcan_peer.py"]) -- "FDCAN3 250 kbit: board status, sensor frames" --- mcu
  ground(["Ground station<br/>ground_peer.py"]) -- "FDCAN3: pad filter, ox injector valve" --- mcu
  card(["SD card<br/>16 MiB blank"]) -- "SDMMC2: logs" --- mcu
  imu(["IMU LSM6DSV32X<br/>lsm6dsv32x_peer.py"]) -- "I2C1 0x6A: accel/gyro, INT PD7" --- mcu
  mag(["Magnetometer IIS2MDC<br/>iis2mdc_peer.py"]) -- "I2C4 0x1E: field, INT PE0" --- mcu
  baro(["Barometer MS5611<br/>ms5611_peer.py"]) -- "I2C5 0x77: PROM, D1 pressure, D2 temp" --- mcu
  adc(["ADC ADS1219<br/>ads1219_peer.py"]) -- "I2C2 0x40: analog channels, INT PE12" --- mcu
  movella(["Movella MTi<br/>movella_peer.py"]) -- "USART3 2 Mbaud: MTData2 at 200 Hz" --- mcu
```

The unedited firmware (FreeRTOS on the STM32H733) boots in Simantic's simulator, `sim`, with the AK45 servo, IMU,
magnetometer, barometer, ADC, Movella MTi and RocketCAN all on its pins. The peers are Python scripts in `sim/`,
modelled at transaction level (the bytes on each bus, not electrical behaviour). The flight in `sim/flight.py` is
1-D vertical kinematics with no drag: the ground peer opens the injector at 2.5 s, and the altitude in the
timeline is decoded by the barometer peer from the pressure bytes the firmware reads back. The injector line is
the ground peer's command on the bus, not a firmware acknowledgement. Install `sim` 0.10.0 or newer with
`curl -fsSL https://simantic.com/install | sh`; PlatformIO is needed for the build. Timing and sensor values are
not measured against hardware, and `sim` is proprietary, not open source; see `sim/README.md`.

## Project Structure

- `src/drivers/`: custom peripheral driver modules
- `src/application/`: high-level application logic modules
- `src/third_party/`: third-party libraries, including CubeMX auto-gen files, `closedrocket` source files, and MATLAB codegen 
- `src/common/`: shared resources specific to this project
- `tests/`: everything for [testing](#Unit-Testing)

## Developer Setup

### 1. Clone repo

- Clone repo and initialize submodules: ` git clone --recurse-submodules https://github.com/waterloo-rocketry/cansw_canards`

### 2. Firmware dev in PlatformIO: edit, build, flash, debug

All firmware dev is done in VSCode using PlatformIO (previously using devcontainer) for easy cross-platform usage.

- Install the PlatformIO extension in VSCode and open this folder
- PlatformIO should auto-detect this project by recognizing the `platformio.ini` file
- Use the PlatformIO tab to Build and Upload. Ex, `Project Tasks > General > Build`
- Use the "Run and Debug" tab to debug on the board once connected via ST-Link
- If needed, switch between debug/release environments: `Ctrl+Shift+P > PlatformIO: Pick Project Environment`
- Auto-format the project code: `Project Tasks > Custom > Format`

### 2.1 STM32CubeMX

Steps to configure stuff in CubeMX and auto-generate code:

- Open `cansw_canard_board_h733.ioc` and `cansw_canard_board_h723.ioc` in CubeMX and configure things as needed
- Click `Generate Code` and wait for the pop-up that indicates it's done
- Run the script `./scripts/cubemx_cleanup.sh` to make the auto-gen files conform to our project setup
- Run clang-format (see section 2) to format the auto-gen code

### 3. Run unit tests in devcontainer

The devcontainer has everything installed to run unit tests using CMake. One could also install all the dependencies locally if desired but the devcontainer is recommended to avoid "but it works on my machine!" issues.

Devcontainer setup:

- [Install devcontainers](https://code.visualstudio.com/docs/devcontainers/tutorial)
- In a new vscode window, use `Ctrl+Shift+P`, search `Dev Containers: Open Folder In Container...`, then select this project folder
    - The first time opening the project will take several minutes to build the devcontainer. Subsequent times will be instant.

Run tests:

- Open the CMake plugin tab from the sidebar
- Under `Configure`, select which build type you want
- Hover over `Build`, click the build icon to build the configuration
    - The build preset should automatically be selected (eg, `Build Firmware (Debug) preset`)

### 4. Build HIL
- Switch to the HIL build environment: `Ctrl+Shift+P > PlatformIO: Pick Project Environment > env:hil`
- Build and upload as usual. Now the board will be in HIL mode


## Unit Testing

We use GoogleTest and [Fake Function Framework (fff)](https://android.googlesource.com/platform/external/fff/+/cacd5f5459dba658e0c88ceff32282a0e9fab0a5/README.md) for unit testing. All testing-related files are in `tests/`.

- Tests are built from `tests/CMakeLists.txt` which is separate from the project's main build config. Building and running tests is done via cmake.
- Test source code should be written in `tests/unit/`.
- Mocks should be made with fff in `tests/mocks/`.

### Add a test

- Add a new test group file in `tests/unit/`. See `test_dummy.cpp` for example of test structure.
- Add the test group to the cmake build system by editing `tests/CMakeLists.txt`:
    - At the bottom of the file, add the new test group using the `add_test_group()` helper.
      (Read the comments + existing examples explaining how it works)

### Add a mock

We do not include the STM32 HAL library nor FreeRTOS when compiling the project for unit tests.
So if a source file uses a HAL or FreeRTOS file, those files and their functions must be mocked using fff.
This also lets us mock any module that a test interacts with but doesn't test.

Example 1:

- `src/drivers/gpio/gpio.c` uses FreeRTOS semaphores via `#include "semphr.h`.
    - In the actual firmware, the real `semphr.h` is included when compiling. But for unit tests, the real `semphr.h` is not included when compiling. So, the unit tests fail to compile (it can't find a `semphr.h` file).
        - To correct this we add a "fake" `semphr.h` in `tests/mocks/semphr.h`. All files in this folder are included when compiling unit test, so the tests now compile.
    - The gpio code uses functions from the real `semphr.h` like `xSemaphoreTake()`. These don't exist in our fake `semphr.h` yet.
        - To correct this we need to create a mock `xSemaphoreTake()` function using fff.
          [fff's Readme](https://github.com/meekrosoft/fff?tab=readme-ov-file#hello-fake-world) describes how to create fake functions. Here's the mock for `xSemaphoreTake()`:

            ```
            // The func to mock: BaseType_t xSemaphoreTake(SemaphoreHandle_t arg0, TickType_t arg1)

            DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
            ```

            First we put the _declaration_ (`DECLARE_FAKE...`) in `mocks/semphr.h`. Then, put the actual _definition_ (`DEFINE_FAKE...`) in the corresponding `mocks/semphr.c`.

- Now in the gpio tests, we can access the mocked semaphore functions via fff to test that the gpio code uses semaphores correctly.

Example 2:

- `src/application/estimator/estimator.c` takes input from flightphase via `#include "application/flight_phase/flight_phase.h`.
    - To test estimator, we will test various inputs from flightphase. But we don't want to be testing flightphase at the same time.
      So, mock flightphase.
    - The real `flight_phase.h` header is built in unit tests, but the source file `flight_phase.c` is not.
        - Since a function _declaration_ for `get_flight_phase()` exists in the header, we can use fff's `DEFINE_FAKE...` to create a
          fake _definition_ of that `get_flight_phase()`. That definition should be in the test .cpp file, or in the mocks folder if it's applicable for wider use.
        - (Note: unlike example 1 where the header isn't included in unit tests, the flightphase header is included so it doesnt need a `DECLARE_FAKE...`)

### Run/debug tests

- Build in vscode using cmake (see step 3 above)
    - The default `Build Unit Tests With Coverage preset` also automatically runs all tests and generates coverage report.
    - View the coverage report html pages in `build/test/coverage_report` in a local browser
- Use the vscode cmake `Launch` and `Debug` tabs to run/debug individual test groups (cmake shows the available test groups)

## Code Standards

This project follows the [team-wide embedded coding standard](https://docs.waterloorocketry.com/general/standards/embedded-coding-standard.html).

- Use the `Format` target via PlatformIO to format via clang-format (see step 2)

- Rocketlib is included at `src/third_party/rocketlib`.
- Developers should be aware of the BARR Standard and make an effort to follow it.

## Adding Log Messages

When adding a new type of data log message, all of the following should be updated:

- src/application/logger/log.h
    - Add a new enum value to `log_data_type_t`:
        ```diff
         typedef enum {
        +    LOG_TYPE_XXX = M(unique_small_integer),
         } log_data_type_t;
        ```
    - Add a struct definition of your message's data fields to `log_data_container_t`:
        ```diff
         typedef union __attribute__((packed)) {
        +    struct __attribute__((packed)) {
        +        uint32_t l;
        +        float f;
        +        // ...
        +    } typename;
         } log_data_container_t;
        ```
- scripts/logparse.py
    - Add a new format spec to the `FORMATS` dict:
        ```diff
         FORMATS = {
        +    M(unique_small_integer): Spec(name, format, [field, ...]),
         }
        ```

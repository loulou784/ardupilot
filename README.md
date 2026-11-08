# ArduPilot on QNX

This branch ports ArduPilot to QNX SDP 8.0 on AArch64. The target board
is IcePilot: a Raspberry Pi 4 with an iCE40UP5K I/O FPGA.

For general ArduPilot documentation, see the
[upstream README](https://github.com/ArduPilot/ardupilot/blob/master/README.md)
and [ardupilot.org](https://ardupilot.org).

**Last updated:** 2026-11-08. Update the tables in the same commit as the
change they describe.

## Status legend

| Status      | Meaning                                                    |
|-------------|------------------------------------------------------------|
| Done        | Implemented and verified                                   |
| Partial     | Works, with the limitations listed in Notes                |
| Stub        | Compiles but does nothing (inherited from `AP_HAL_Empty`)  |
| Not started | No work yet                                                |
| Not planned | Not needed on this platform                                |

## 1. SITL on QNX

SITL (Software In The Loop) runs ArduPilot with its built-in physics
simulator. Running it on QNX tests the toolchain, the C library and the
operating system with a vehicle that flies, before any hardware driver
exists. Target: the QNX 8.0 AArch64 QEMU VM.

| Step                                                          | Status      | Notes |
|---------------------------------------------------------------|-------------|-------|
| Configure `--board sitl --toolchain ntoaarch64`               | Partial     | Works only when `-D_QNX_SOURCE` and `-lsocket` are passed through environment variables |
| QNX flags in the SITL board (`_QNX_SOURCE`, `socket`, CAN off) | Not started | Move into `boards.py`, shared with the `qnx` board |
| POSIX filesystem `dirent` adapter                             | Not started | **Current blocker:** the build stops at 429/1430 because QNX's `struct dirent` has no `d_type` |
| `AP_HAL_SITL` compiles                                        | Not started | Expect small `__QNX__` guards next to the existing `__APPLE__` ones |
| `SITL` physics library compiles                               | Not started |       |
| `arducopter` links                                            | Not started |       |
| Runs on the QNX VM                                            | Not started |       |
| MAVLink over TCP from the host                                | Not started | Connect MAVProxy or QGroundControl to `<target-ip>:5760` |
| AUTO mission flown                                            | Not started | Completion criterion for this table |

## 2. QNX HAL peripherals (`AP_HAL_QNX`)

Generic QNX support that any real board needs. Built with `--board qnx`.

| Component                            | Status      | Notes |
|--------------------------------------|-------------|-------|
| waf `qnx` board                      | Done        | `ntoaarch64` GCC 12.2, defines `_QNX_SOURCE`, links `socket` and `m` |
| HAL class (`HAL_QNX_Class`)          | Done        | Matches the current `AP_HAL::HAL` constructor |
| System time and panic (`system.cpp`) | Done        | Uses `CLOCK_MONOTONIC` |
| Semaphores                           | Partial     | pthread mutex and condition variable. To do: monotonic timeouts (`pthread_mutex_timedlock_monotonic`, `pthread_condattr_setclock`) instead of `CLOCK_REALTIME` and 200 µs polling |
| Scheduler                            | Stub        | **Current blocker:** no `thread_create`, so startup stops at `Failed to start Logger IO thread`. To do: pthreads with `SCHED_FIFO` priorities, timer and IO threads |
| Filesystem                           | Partial     | Uses ArduPilot's fixed-size `struct dirent`. No writable backend yet, so scripting and file logging are off |
| Storage (parameters)                 | Stub        | To do: file-backed storage, as in `AP_HAL_Linux` |
| UARTDriver                           | Stub        | To do: termios serial ports, plus UDP and TCP for MAVLink |
| Util                                 | Partial     | Hardware RTC is a stub. To do: `clock_settime`, memory information |
| I2C                                  | Stub        | To do: on the QNX Raspberry Pi 4 I2C driver |
| SPI                                  | Stub        | To do: on the QNX Raspberry Pi 4 SPI driver |
| GPIO                                 | Stub        |       |
| RCInput                              | Stub        | IcePilot reads RC input through the FPGA, see table 3 |
| RCOutput                             | Stub        | IcePilot drives outputs through the FPGA, see table 3 |
| AnalogIn                             | Stub        |       |
| DSP (gyro FFT)                       | Stub        | Returns no results |
| CAN                                  | Not planned | Disabled for QNX |
| WSPI, Flash, OpticalFlow             | Not planned | Not used on this platform |

## 3. IcePilot board

| Item                                                     | Status      | Notes |
|----------------------------------------------------------|-------------|-------|
| waf `icepilot` board and `HAL_BOARD_SUBTYPE_QNX_ICEPILOT` | Done        | Subclass of the `qnx` board |
| `arducopter` builds                                      | Done        | QNX AArch64 executable, about 3.4 MB with debug information |
| Runs on the QNX AArch64 QEMU VM                          | Partial     | The process starts, then stops at the Scheduler blocker in table 2 |
| QNX image for the Raspberry Pi 4                         | Not started |       |
| FPGA register and protocol specification                 | Not started | Needed before any FPGA driver |
| FPGA bitstream loading                                   | Not started |       |
| Servo and motor outputs through the FPGA                 | Not started |       |
| RC input through the FPGA                                | Not started |       |
| Sensors (IMU, barometer, compass)                        | Not started | Sensor list to be defined |
| Board configuration (buses, devices, pin map)            | Not started |       |
| Power monitoring and analog inputs                       | Not started |       |
| LEDs, buzzer and safety switch (if fitted)               | Not started |       |
| Start at boot (QNX SLM) as a non-root user               | Not started |       |
| Bench tests (outputs, failsafes, timing jitter)          | Not started | Required before the first flight |
| First flight                                             | Not started |       |

## Building

Requirements:

- QNX SDP 8.0 with the AArch64 target
- Python 3.10 or later with `empy==3.3.4`, `future`, `pexpect` and `dronecan`

```bash
source <path-to-sdp>/qnxsdp-env.sh
./waf configure --board icepilot    # or --board qnx
./waf copter
```

The executable is written to `build/icepilot/bin/arducopter`.

## Running on a QNX target

Upload and start the executable through qconn (port 8000) with the SDP
debugger:

```bash
source <path-to-sdp>/qnxsdp-env.sh
ntoaarch64-gdb --batch \
  -ex "target qnx <target-ip>:8000" \
  -ex "file build/icepilot/bin/arducopter" \
  -ex "upload build/icepilot/bin/arducopter /tmp/arducopter" \
  -ex "set nto-cwd /tmp" \
  -ex run
```

## License

The ArduPilot project is licensed under the GNU General Public
License, version 3 or (at your option) any later version
(SPDX identifier `GPL-3.0-or-later`).

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

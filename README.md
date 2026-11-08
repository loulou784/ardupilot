# ArduPilot on QNX

This branch ports ArduPilot to QNX SDP 8.0 on AArch64. The target board
is IcePilot: a Raspberry Pi 4 with an iCE40UP5K I/O FPGA.

For general ArduPilot documentation, see the
[upstream README](https://github.com/ArduPilot/ardupilot/blob/master/README.md)
and [ardupilot.org](https://ardupilot.org).

**Last updated:** 2026-11-08. Update the tables in the same commit as the
change they describe.

## Status legend

Steps are numbered by table: 2.5 is the fifth row of table 2. Use these
numbers to refer to a step in commits and issues.

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

| #    | Step                                                          | Status      | Notes |
|------|---------------------------------------------------------------|-------------|-------|
| 1.1  | Configure `--board sitl --toolchain ntoaarch64`               | Partial     | Works only when `-D_QNX_SOURCE` and `-lsocket` are passed through environment variables |
| 1.2  | QNX flags in the SITL board (`_QNX_SOURCE`, `socket`, CAN off) | Not started | Move into `boards.py`, shared with the `qnx` board |
| 1.3  | POSIX filesystem `dirent` adapter                             | Not started | **Current blocker:** the build stops at 429/1430 because QNX's `struct dirent` has no `d_type` |
| 1.4  | `AP_HAL_SITL` compiles                                        | Not started | Expect small `__QNX__` guards next to the existing `__APPLE__` ones |
| 1.5  | `SITL` physics library compiles                               | Not started |       |
| 1.6  | `arducopter` links                                            | Not started |       |
| 1.7  | Runs on the QNX VM                                            | Not started |       |
| 1.8  | MAVLink over TCP from the host                                | Not started | Connect MAVProxy or QGroundControl to `<target-ip>:5760` |
| 1.9  | AUTO mission flown                                            | Not started | Completion criterion for this table |

## 2. QNX HAL peripherals (`AP_HAL_QNX`)

Generic QNX support that any real board needs. Built with `--board qnx`.

| #    | Component                            | Status      | Notes |
|------|--------------------------------------|-------------|-------|
| 2.1  | waf `qnx` board                      | Done        | `ntoaarch64` GCC 12.2, defines `_QNX_SOURCE`, links `socket` and `m` |
| 2.2  | HAL class (`HAL_QNX_Class`)          | Done        | Matches the current `AP_HAL::HAL` constructor |
| 2.3  | System time and panic (`system.cpp`) | Done        | Uses `CLOCK_MONOTONIC` |
| 2.4  | Semaphores                           | Partial     | pthread mutex and condition variable. To do: monotonic timeouts (`pthread_mutex_timedlock_monotonic`, `pthread_condattr_setclock`) instead of `CLOCK_REALTIME` and 200 µs polling |
| 2.5  | Scheduler                            | Stub        | **Current blocker:** no `thread_create`, so startup stops at `Failed to start Logger IO thread`. To do: pthreads with `SCHED_FIFO` priorities, timer and IO threads |
| 2.6  | Filesystem                           | Partial     | Uses ArduPilot's fixed-size `struct dirent`. No writable backend yet, so scripting and file logging are off. The POSIX `dirent` adapter (1.3) unblocks this |
| 2.7  | Storage (parameters)                 | Stub        | To do: file-backed storage, as in `AP_HAL_Linux` |
| 2.8  | UARTDriver                           | Stub        | To do: termios serial ports, plus UDP and TCP for MAVLink |
| 2.9  | Util                                 | Partial     | Hardware RTC is a stub. To do: `clock_settime`, memory information |
| 2.10 | I2C                                  | Stub        | To do: on the QNX Raspberry Pi 4 I2C driver |
| 2.11 | SPI                                  | Stub        | To do: on the QNX Raspberry Pi 4 SPI driver |
| 2.12 | GPIO                                 | Stub        |       |
| 2.13 | RCInput                              | Stub        | IcePilot reads RC input through the FPGA, see 3.8 |
| 2.14 | RCOutput                             | Stub        | IcePilot drives outputs through the FPGA, see 3.7 |
| 2.15 | AnalogIn                             | Stub        |       |
| 2.16 | DSP (gyro FFT)                       | Stub        | Returns no results |
| 2.17 | CAN                                  | Not planned | Disabled for QNX |
| 2.18 | WSPI, Flash, OpticalFlow             | Not planned | Not used on this platform |

## 3. IcePilot board

| #    | Item                                                     | Status      | Notes |
|------|----------------------------------------------------------|-------------|-------|
| 3.1  | waf `icepilot` board and `HAL_BOARD_SUBTYPE_QNX_ICEPILOT` | Done        | Subclass of the `qnx` board |
| 3.2  | `arducopter` builds                                      | Done        | QNX AArch64 executable, about 3.4 MB with debug information |
| 3.3  | Runs on the QNX AArch64 QEMU VM                          | Partial     | The process starts, then stops at the Scheduler blocker (2.5) |
| 3.4  | QNX image for the Raspberry Pi 4                         | Not started |       |
| 3.5  | FPGA register and protocol specification                 | Not started | Needed before 3.6 to 3.8 |
| 3.6  | FPGA bitstream loading                                   | Not started |       |
| 3.7  | Servo and motor outputs through the FPGA                 | Not started |       |
| 3.8  | RC input through the FPGA                                | Not started |       |
| 3.9  | Sensors (IMU, barometer, compass)                        | Not started | Sensor list to be defined |
| 3.10 | Board configuration (buses, devices, pin map)            | Not started |       |
| 3.11 | Power monitoring and analog inputs                       | Not started |       |
| 3.12 | LEDs, buzzer and safety switch (if fitted)               | Not started |       |
| 3.13 | Start at boot (QNX SLM) as a non-root user               | Not started |       |
| 3.14 | Bench tests (outputs, failsafes, timing jitter)          | Not started | Required before the first flight |
| 3.15 | First flight                                             | Not started |       |

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

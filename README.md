# ArduPilot on QNX

This branch ports ArduPilot to QNX SDP 8.0 on AArch64. The target board
is IcePilot: a Raspberry Pi 4 with an iCE40UP5K I/O FPGA.

For general ArduPilot documentation, see the
[upstream README](https://github.com/ArduPilot/ardupilot/blob/master/README.md)
and [ardupilot.org](https://ardupilot.org).

**Last updated:** 2026-11-08. Update the tables in a `README.md:` commit
at the end of each step.

## Approach

The port follows `AP_HAL_Linux` and the Linux boards as closely as
possible. Backends are ported from `AP_HAL_Linux` file by file: one
commit copies the Linux file, the next adapts it to QNX, so each change
can be reviewed against Linux. QNX code differs from Linux only where
QNX itself differs; those places are listed in
[Differences from the Linux port](#differences-from-the-linux-port).

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
| 1.1  | `./waf configure --board SITL_aarch64_qnx`                    | Done        | Defined like `SITL_arm_linux_gnueabihf`, but linked dynamically. Configure needs no options or environment variables, and every configure check gives the correct result for QNX |
| 1.2  | QNX build flags (`_QNX_SOURCE`, `libsocket`, CAN)             | Done        | `_QNX_SOURCE` for every QNX build, as g++ defines `_GNU_SOURCE` on Linux. A `libsocket` configure check modelled on the `librt` one. SocketCAN decided by the target OS, so SITL on QNX uses multicast CAN, as on macOS |
| 1.3  | POSIX filesystem `dirent` adapter                             | Not started | **Current blocker.** QNX's `struct dirent` has no `d_type` (`AP_Filesystem.cpp`, `AP_Filesystem_Sys.cpp`, `AP_Filesystem_ROMFS.cpp`, `GCS_FTP.cpp`), and `AP_Filesystem_posix.cpp` uses the Linux `statfs` from `sys/vfs.h` |
| 1.4  | `AP_HAL_SITL` compiles                                        | Not started | 4 files fail: `UART_utils.cpp` uses the Linux `asm/ioctls.h`, `UARTDriver.cpp` uses `CRTSCTS` (QNX has `IHFLOW` and `OHFLOW`), and `RCInput.cpp` and `Scheduler.cpp` hit the `htole16` fallback in `AP_HAL/utility/sparse-endian.h`, which clashes with QNX's `net/netbyte.h` |
| 1.5  | `SITL` physics library and other libraries compile            | Partial     | The physics library compiles unchanged. lwIP (used by AP_Networking) redefines `struct iovec`, which QNX already defines |
| 1.6  | `arducopter` links                                            | Not started |       |
| 1.7  | Runs on the QNX VM                                            | Not started |       |
| 1.8  | MAVLink over TCP from the host                                | Not started | Connect MAVProxy or QGroundControl to `<target-ip>:5760` |
| 1.9  | AUTO mission flown                                            | Not started | Completion criterion for this table |

## 2. QNX HAL peripherals (`AP_HAL_QNX`)

Generic QNX support that any real board needs. Built with `--board qnx`.

| #    | Component                            | Status      | Notes |
|------|--------------------------------------|-------------|-------|
| 2.1  | QNX boards                           | Partial     | Python classes `qnx` and `icepilot` for now, using `ntoaarch64` GCC 12.2. To do, as for `LinuxBoard`: a `QNXBoard` class with boards in `libraries/AP_HAL_QNX/hwdef/`, the malloc wrapper that zeroes allocations, and `-pthread` |
| 2.2  | HAL class (`HAL_QNX_Class`)          | Done        | Matches the current `AP_HAL::HAL` constructor |
| 2.3  | System time and panic (`system.cpp`) | Partial     | Same as `AP_HAL_Linux`, using `CLOCK_MONOTONIC`. To do: the stopped-clock support, with the Scheduler (2.5) |
| 2.4  | Semaphores                           | Done        | Same as `AP_HAL_Linux`: pthread mutex and condition variable |
| 2.5  | Scheduler                            | Stub        | **Current blocker:** no `thread_create`, so startup stops at `Failed to start Logger IO thread`. To do: port `Scheduler` and `Thread` from `AP_HAL_Linux` |
| 2.6  | Filesystem                           | Partial     | Uses ArduPilot's fixed-size `struct dirent`. No writable backend yet, so scripting and file logging are off. The POSIX `dirent` adapter (1.3) unblocks this |
| 2.7  | Storage (parameters)                 | Stub        | To do: port the file-backed `Storage` from `AP_HAL_Linux` |
| 2.8  | UARTDriver                           | Stub        | To do: port `UARTDriver` and its serial, UDP and TCP devices from `AP_HAL_Linux` |
| 2.9  | Util                                 | Partial     | Hardware RTC is a stub. To do: port `Util` from `AP_HAL_Linux` |
| 2.10 | I2C                                  | Stub        | To do: the `AP_HAL_Linux` device manager, on the QNX Raspberry Pi 4 I2C driver instead of `i2c-dev` |
| 2.11 | SPI                                  | Stub        | To do: the `AP_HAL_Linux` device manager, on the QNX Raspberry Pi 4 SPI driver instead of `spidev` |
| 2.12 | GPIO                                 | Stub        | To do: on the QNX GPIO driver instead of sysfs and `/dev/mem` |
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
| 3.9  | Sensors (IMU, barometer, compass)                        | Not started | Sensor list to be defined, then declared in the board hwdef (3.10) |
| 3.10 | Board configuration (buses, devices, pin map)            | Not started | In `libraries/AP_HAL_QNX/hwdef/icepilot/hwdef.dat`, as for Linux boards such as `navio2` |
| 3.11 | Power monitoring and analog inputs                       | Not started |       |
| 3.12 | LEDs, buzzer and safety switch (if fitted)               | Not started |       |
| 3.13 | Start at boot (QNX SLM) as a non-root user               | Not started |       |
| 3.14 | Bench tests (outputs, failsafes, timing jitter)          | Not started | Required before the first flight |
| 3.15 | First flight                                             | Not started |       |

## Differences from the Linux port

Each of these follows from a real difference between QNX and Linux, and
is commented where it appears in the code. For steps not done yet, the
QNX column is the planned approach.

| Area                     | Linux port                       | QNX port                                    | Reason |
|--------------------------|----------------------------------|---------------------------------------------|--------|
| C library                | glibc                            | Defines `_QNX_SOURCE`, links `libsocket`    | g++ defines `_GNU_SOURCE` on Linux but not `_QNX_SOURCE` on QNX, without which QNX hides many POSIX declarations. QNX keeps sockets in a separate library |
| Malloc wrapper           | Added before the configure checks | Added after the configure checks            | QNX has no symbol versioning, so `--wrap,malloc` also catches the `malloc` that `libc++.so` uses and the check programs fail to link |
| SITL linking             | Static                           | Dynamic                                     | The QNX SDP has no static `libsocket` |
| Directory entries        | System `struct dirent`           | ArduPilot's fixed-size `struct dirent`      | QNX's `struct dirent` has a variable-length name and no `d_type` |
| Event loop (`Poller`)    | `epoll`, `eventfd`, `timerfd`    | `poll()` or QNX pulses                      | Not available on QNX |
| SPI and I2C              | `spidev`, `i2c-dev`              | QNX SPI and I2C drivers                     | Different driver interfaces |
| GPIO and PWM             | sysfs, `/dev/mem`                | QNX GPIO driver, `mmap_device_memory()`     | No sysfs on QNX |
| CAN                      | SocketCAN                        | SITL: multicast CAN, as on macOS. HAL: off for now | No SocketCAN on QNX |

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

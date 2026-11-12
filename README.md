# ArduPilot on QNX

This branch ports ArduPilot to QNX SDP 8.0 on AArch64, and benchmarks it
against the regular Linux build of ArduPilot. The target board is
IcePilot: a Raspberry Pi 4 with an iCE40UP5K I/O FPGA.

For general ArduPilot documentation, see the
[upstream README](https://github.com/ArduPilot/ardupilot/blob/master/README.md)
and [ardupilot.org](https://ardupilot.org).

**Last updated:** 2026-11-09. Update the tables in a `README.md:` commit
at the end of each step.

## Approach

**Shared code** (SITL, `AP_Filesystem`, the build system and other code
every platform compiles) follows the Linux port as closely as possible.
It differs from Linux only where QNX itself differs, and those places
are listed in
[Differences from the Linux port](#differences-from-the-linux-port).

**`AP_HAL_QNX`** is optimized for QNX, and the device is benchmarked
against the Linux build (table 4). It is built in four stages: port,
measure, optimize, measure.

1. Each backend is ported from `AP_HAL_Linux` file by file, in one
   commit that builds; `git show -B -C -C <commit>` shows it as a copy
   of the Linux files plus the QNX changes. The first QNX HAL therefore
   has Linux's design, and is a like-for-like baseline.
2. QNX-native optimizations follow as separate commits, each behind a
   build option, so every gain can be measured on its own (2.19 onward).

The `AP_HAL` interfaces stay the same as on Linux, so everything above
the HAL is the same code on both systems, and board drivers are written
against the `AP_HAL` device interfaces so that they run under both.

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

**Complete:** SITL on QNX builds, runs and flies an AUTO mission.

| #    | Step                                                          | Status      | Notes |
|------|---------------------------------------------------------------|-------------|-------|
| 1.1  | `./waf configure --board SITL_aarch64_qnx`                    | Done        | Defined like `SITL_arm_linux_gnueabihf`, but linked dynamically. Configure needs no options or environment variables, and every configure check gives the correct result for QNX |
| 1.2  | QNX build flags (`_QNX_SOURCE`, `libsocket`, CAN)             | Done        | `_QNX_SOURCE` for every QNX build, as g++ defines `_GNU_SOURCE` on Linux. A `libsocket` configure check modelled on the `librt` one. SocketCAN decided by the target OS, so SITL on QNX uses multicast CAN, as on macOS |
| 1.3  | POSIX filesystem `dirent` adapter                             | Done        | Every QNX target uses ArduPilot's fixed-size `struct dirent`. The POSIX backend copies each entry from QNX's and takes its type from `fstatat()`, as the FATFS backend does. Disk space uses `statvfs()`. Directory listing verified at runtime over MAVLink FTP (1.8) |
| 1.4  | `AP_HAL_SITL` compiles                                        | Done        | Serial speed through standard `termios`, `CRTSCTS` defined as `IHFLOW \| OHFLOW`, and `O_SYNC` without `O_DIRECT`, as on OpenBSD. `sparse-endian.h` uses QNX's byte order functions |
| 1.5  | `SITL` physics library and other libraries compile            | Done        | The physics library compiles unchanged. lwIP (used by AP_Networking) uses QNX's `struct iovec`, set in `lwipopts.h` next to the existing macOS settings |
| 1.6  | `arducopter` links                                            | Done        | QNX AArch64 executable, about 6.4 MB with debug information, linked with `libsocket`. The malloc wrapper is in and waf's symbol check passes. One warning, in seven Lua files: Lua's `SIZE_C` replaces the `SIZE_C()` macro from QNX's `stdint.h`. Harmless, as Lua uses only its own; to be fixed in `AP_Scripting` |
| 1.7  | Runs on the QNX VM                                            | Done        | Starts and runs the Copter main loop on the QNX 8.0 AArch64 QEMU VM: EKF3 aligned, GPS fix, `ArduPilot Ready`. Simulated time runs at about 0.3x real time, as the VM is emulated (no KVM for AArch64 on an x86 host) |
| 1.8  | MAVLink over TCP from the host                                | Done        | pymavlink on the host, connected to `<target-ip>:5760`: heartbeat, 30 telemetry message types, parameter reads, and MAVLink FTP listings of real QNX files and `@SYS` |
| 1.9  | AUTO mission flown                                            | Done        | Flown as ArduPilot's autotest does (`AUTO_OPTIONS` 3, arm in AUTO): takeoff to 20 m, a 50 m square, RTL. Every waypoint reached in order, landed and disarmed 0.0 m from home, no failsafe or EKF messages. A 6.5 MB flight log was written to `logs/` on QNX |

## 2. QNX HAL peripherals (`AP_HAL_QNX`)

Generic QNX support that any real board needs. Built with `--board qnx`.
Rows 2.1 to 2.18 port `AP_HAL_Linux`; rows 2.19 onward are QNX-native
optimizations, each behind a build option and measured in table 4.

| #    | Component                            | Status      | Notes |
|------|--------------------------------------|-------------|-------|
| 2.1  | QNX boards                           | Done        | `QNXBoard`, like `LinuxBoard`: each directory in `libraries/AP_HAL_QNX/hwdef/` (`qnx`, `icepilot`) is a board, and `qnx_hwdef.py` turns its `hwdef.dat` into `hwdef.h`. Same build settings as Linux: `-O3`, `-pthread`, the malloc wrapper that zeroes allocations, and `--apstatedir`. Toolchain `ntoaarch64`, GCC 12.2 |
| 2.2  | HAL class (`HAL_QNX_Class`)          | Done        | `run()` as in `AP_HAL_Linux`: the same driver start order, with the scheduler threads released before `setup()`; 10 serial ports; all of Linux's command-line options (`--serial0` to `--serial9`, log, terrain and storage directories, defaults file, CPU affinity); and a clean exit on SIGINT and SIGTERM, tested with simulation on hardware (2.24): exit code 0 within a second |
| 2.3  | System time and panic (`system.cpp`) | Done        | Same as `AP_HAL_Linux`: `CLOCK_MONOTONIC`, and the stopped clock that Replay uses |
| 2.4  | Semaphores                           | Done        | Same as `AP_HAL_Linux`: pthread mutex and condition variable. Optimization: 2.23 |
| 2.5  | Scheduler                            | Done        | Ported from `AP_HAL_Linux`: the timer (1 kHz), UART (100 Hz), RC input (100 Hz) and IO (50 Hz) threads, the same `SCHED_FIFO` priorities, and `thread_create()`. QNX differences: realtime scheduling without root, CPU affinity as a runmask, and stack bounds from thread local storage. Startup now gets to `AP_Baro::init()` and stops at the missing-barometer error, as `AP_HAL_Linux` does on a board with no sensors. Optimization: 2.19 |
| 2.6  | Filesystem                           | Done        | The POSIX backend, as on Linux, with ArduPilot's fixed-size `struct dirent` (1.3). Tested on the Raspberry Pi 4 over MAVFTP: real directories listed; a directory created, a 50 kB file written and read back unchanged, then both deleted. File logging and scripting are on by default, as on Linux. With simulation on hardware (2.24), a flight writes a 5 MB `.BIN` log to `/var/lib/ardupilot/logs`, and a Lua script in `./scripts` runs |
| 2.7  | Storage (parameters)                 | Done        | Ported from `AP_HAL_Linux`: a 16 kB file, `ArduCopter.stg`, in `/var/lib/ardupilot`. As on Linux, the directory must exist and be writable by the user ArduPilot runs as (see 3.13). QNX differences: the directory is opened with `O_DIRECTORY`, which QNX's `openat()` needs, and no glibc extensions (`strndupa()`, `%m`). Tested on the Raspberry Pi 4: a parameter set over MAVLink survives a restart |
| 2.8  | UARTDriver                           | Done        | Ported from `AP_HAL_Linux`, with its console, serial port, UDP and TCP devices. QNX difference: serial ports are set up with standard termios. Tested on the Raspberry Pi 4: console output; MAVLink over TCP, with telemetry and the full parameter list; MAVLink over a serial port, TELEM1 (`/dev/ser3`) at 115200 baud, to QGroundControl through a USB to UART adapter. As on Linux, while startup is stopped at a configuration error, only SERIAL0 is served. Not tested yet: UDP |
| 2.9  | Util                                 | Done        | Ported from `AP_HAL_Linux`: the system clock as the hardware RTC, custom directories and defaults file, system ID, random values, file helpers. Left out, as only Linux drivers use them: the sysfs buzzer, the PWM IMU heater and the board detection from `/proc/cpuinfo`. Tested on the Raspberry Pi 4: `-s`, `--defaults` and `-c` work; with `BRD_RTC_TYPES` allowing it, `SYSTEM_TIME` matches the Pi's clock. As on Linux, `--defaults` has no short `-d` form |
| 2.10 | I2C                                  | Done        | Ported from `AP_HAL_Linux`, on the QNX I2C drivers (`/dev/i2c<n>`) instead of `i2c-dev`: one `devctl()` per transfer, from and to the caller's buffers. Device callbacks run on one thread per bus (`DeviceBus`, see the differences below). As on ChibiOS, a bus runs at the speed of its slowest device. Tested on the Raspberry Pi 4 with `examples/BusTest`: a probe of an absent address takes 65 µs, and two callbacks on bus 1, every 1000 and 2500 µs, each with a transfer, run within 1 µs of their period 98% of the time. With an MPU-6500 on I2C1 (pins 3 and 5), then on I2C6 (pins 15 and 16): found at 0x68 by the scan, and its ID registers (WHO_AM_I 0x70) read with a repeated start in 130 µs at 400 kHz, with no failed read in 7 runs. Needs the fixed `i2c-bcm2711` (3.4) |
| 2.11 | SPI                                  | Partial     | Ported from `AP_HAL_Linux`, on io-spi instead of `spidev`: the board's `QNX_SPIDEV` lines give each device's io-spi node, and an exchange is one `devctl()`, from and to the caller's buffers. A write then a read is one exchange, with the chip select held throughout. Works around the shared bus clock of `spi-bcm2711`, measured: a device used after one at another rate ran at the other's rate. Tested on the Raspberry Pi 4 with `examples/BusTest`, with nothing on the bus: a 2-byte exchange at 10 MHz takes 26 µs; callbacks every 1000 µs on two devices run within 1 µs of their period 98% of the time. With an MPU-6500 on each chip select in turn, SPI0 CE0 and CE1 in mode 3, then SPI6 CE0 and CE1 in mode 0 (a temporary hwdef with the SPI6 nodes): WHO_AM_I 0x70 on that chip select only, and read every 1000 µs at 1 MHz, 2000 times per run, with no failed or wrong read; SPI0 and SPI6 callbacks run side by side, on their own threads. Needs the fixed `spi-bcm2711` (3.4). To do: a device read at 10 MHz (the IMU driver, 3.9). Optimization: 2.21 |
| 2.12 | GPIO                                 | Partial     | `GPIO_RPI`, as on Linux, through the `rpi_gpio` resource manager of the QNX Raspberry Pi images: about 5 µs per operation. Unlike Linux, pin interrupts work: `rpi_gpio` sends a pulse on each edge. A board lists the pins ArduPilot may use (`HAL_QNX_GPIO_PIN_MASK`), so that a parameter cannot take over a bus pin. Tested on the Raspberry Pi 4 with `examples/GPIOTest` on GPIO26: writes and toggles behave; 200 edges with interrupts on both edges, none missed, all with the right level, 11.5 µs after the write on average; none after detaching. To do: test with devices |
| 2.13 | RCInput                              | Partial     | Through `AP_RCProtocol`, as on ChibiOS: a receiver on a serial port set to RCIN (`SERIALn_PROTOCOL 23`), eg. CRSF or ELRS, SBUS through an inverter, DSM, SRXL. With no receiver there are no channels, so the RC failsafe works. Tested on the Raspberry Pi 4 without a receiver, and with simulation on hardware (2.24). To do: test with a receiver. IcePilot's RC input pin, through the FPGA: 3.8 |
| 2.14 | RCOutput                             | Stub        | IcePilot drives outputs through the FPGA, see 3.7 |
| 2.15 | AnalogIn                             | Stub        |       |
| 2.16 | DSP (gyro FFT)                       | Stub        | Returns no results |
| 2.17 | CAN                                  | Not planned | Disabled for QNX |
| 2.18 | WSPI, Flash, OpticalFlow             | Not planned | Not used on this platform |
| 2.19 | Optimization: scheduler timers       | Done        | Sleeps use QNX 8 high-resolution timers: one per thread, with a 1 µs tolerance, sending a pulse. QNX's `nanosleep()` ends on the 1 ms tick, and QNX 8 cannot change the tick at run time (`ClockPeriod()` reports `ENOTSUP`, even for root). Measured on the Raspberry Pi 4 with `examples/SchedulerTiming`: `delay_microseconds()` of 50, 250, 1000 and 2500 µs takes 55, 255, 1005 and 2506 µs, where it took 1000, 1000, 2000 and 3000 µs; the timer thread keeps 1000 Hz either way. Needs the `high_resolution_timer` ability: root has it, another user gets it with `on -A nonroot,allow,high_resolution_timer -u <user>`. Without it, sleeps fall back to the tick, with a warning. `AP_HAL_QNX_HIGH_RES_TIMER_ENABLED 0` builds the baseline. Timer thread periods with high-resolution timers: 98% between 996 and 1004 µs, 99.9% under 1008 µs; so a periodic timer for the scheduler threads would gain nothing, and the rare outliers are left to the priority work (3.13). In the main loop, with simulation on hardware (2.24) at 400 Hz: 0.2 long loops per 4000, where the 1 ms tick gives 513 (13%), and its shortest worst-case loop is 3.0 ms. The pulse channels have a fixed priority (`_NTO_CHF_FIXED_PRIORITY`): without it, receiving the pulse dropped every scheduler thread to priority 10 |
| 2.20 | Optimization: event loop             | Not started | Pulses and `ionotify` instead of the ported `poll()` loop. Measure: wake-up latency |
| 2.21 | Optimization: IMU SPI path           | Not started | Direct SPI controller access with interrupts handled in ArduPilot, instead of going through the SPI driver. Measure: transfer time and IMU sample regularity |
| 2.22 | Optimization: CPU partitioning       | Not started | Adaptive partitioning, guaranteeing the flight code a CPU budget even when the system is overloaded. Measure: loop timing with a background CPU hog |
| 2.23 | Optimization: semaphore timeouts     | Done        | `Semaphore::take()` blocks in `pthread_mutex_timedlock_monotonic()`, where Linux polls every 200 µs; `BinarySemaphore` times out on `CLOCK_MONOTONIC`, where Linux uses `CLOCK_REALTIME`, which jumps when GPS time sets the clock; with the `high_resolution_timer` ability, timeouts get a 1 µs tolerance with `TimerTimeout(TIMER_TOLERANCE)`, as kernel timeouts otherwise end on the 1 ms tick. Measured on the Raspberry Pi 4 with `examples/SchedulerTiming`: `take()` wakes 5.1 µs after the semaphore is given, where it took 115.7 µs; `take(1)` times out after 1007 µs, was 1040 µs; `BinarySemaphore::wait(500)` after 505.5 µs, was 966.7 µs |
| 2.24 | Simulation on hardware               | Done        | Board `qnx-SimOnHardWare`: the whole vehicle on `AP_HAL_QNX` on a Raspberry Pi 4, with simulated sensors, GPS, RC and physics, as the ChibiOS SimOnHardWare boards; nothing attached to the Raspberry Pi. On the Raspberry Pi 4 it runs at 1.00 times real time and flies the 1.9 AUTO mission test: take off to 20 m, a 50 m square and RTL, landing back home. Its 400 Hz main loop, from the `PM` log messages: 0.2 long loops per 4000, worst loop 2.5 ms, 33% load. The first like-for-like workload for table 4 |

## 3. IcePilot board

| #    | Item                                                     | Status      | Notes |
|------|----------------------------------------------------------|-------------|-------|
| 3.1  | waf `icepilot` board and `HAL_BOARD_SUBTYPE_QNX_ICEPILOT` | Done        | `libraries/AP_HAL_QNX/hwdef/icepilot/hwdef.dat`, which sets the subtype |
| 3.2  | `arducopter` builds                                      | Done        | QNX AArch64 executable, about 4.2 MB with debug information and scripting |
| 3.3  | Runs on the QNX AArch64 QEMU VM                          | Partial     | Starts its threads, then stops at the missing-barometer error (sensors: 3.9). The same on a Raspberry Pi 4 with the QNX 8.0 quick start image, run as an ordinary user, with the Linux `SCHED_FIFO` priorities: main 12, timer 15, UART 14, RC input 13, IO 10. Since 2.8 the error is shown on the console and sent over MAVLink |
| 3.4  | QNX image for the Raspberry Pi 4                         | Partial     | The IcePilot boot image is built, outside this repository, from the QNX 8.0 quick start image, with the IcePilot pin layout. It also replaces the board's SPI and I2C drivers with fixed builds from the BSP sources: both started a transfer, then waited for the interrupt thread's signal without checking whether it had come already, so a signal sent first was lost. With the stock `spi-bcm2711`, nearly every exchange of up to 64 bytes failed with `ETIMEDOUT` after 1 ms; the I2C driver had the same race, which shows under load. The SPI driver also kept the bus clock of the last device configured, and its timeout grew with the square of the length. To do: install the rebuilt image; until then, the fixed drivers are started by hand |
| 3.5  | FPGA register and protocol specification                 | Not started | Needed before 3.6 to 3.8 |
| 3.6  | FPGA bitstream loading                                   | Not started |       |
| 3.7  | Servo and motor outputs through the FPGA                 | Not started |       |
| 3.8  | RC input through the FPGA                                | Not started |       |
| 3.9  | Sensors (IMU, barometer, compass)                        | Partial     | Declared in the hwdef, from the IcePilot pin layout: ICM-42688-P on SPI0 CE0, a second InvenSense v3 IMU on SPI0 CE1, the BMP390 barometer on I2C1, compasses probed on I2C6. To do: test with the sensors, and set their rotations |
| 3.10 | Board configuration (buses, devices, pin map)            | Partial     | In `libraries/AP_HAL_QNX/hwdef/icepilot/hwdef.dat`, as for Linux boards such as `navio2`: I2C buses (I2C1 internal, I2C6 external), SPI devices on the io-spi nodes of the boot image, and the six pins ArduPilot may use as GPIO. To do: sensor rotations, FPGA, CAN and the FPGA flash |
| 3.11 | Power monitoring and analog inputs                       | Not started |       |
| 3.12 | LEDs, buzzer and safety switch (if fitted)               | Not started |       |
| 3.13 | Start at boot (QNX SLM) as a non-root user               | Not started | With restart on failure (SLM or the High Availability Manager). The image must create `/var/lib/ardupilot`, owned by that user, and start ArduPilot with the `high_resolution_timer` ability (2.19). For CPU affinity to a set of CPUs other than one or all, startup must define it as a cluster, eg. `-c pair:0xc`. Measure restart time in table 4 |
| 3.14 | Bench tests (outputs, failsafes, timing jitter)          | Not started | Required before the first flight |
| 3.15 | First flight                                             | Not started |       |

## 4. Benchmark against Linux

The same IcePilot hardware runs the regular Linux build of ArduPilot
and the QNX build, so that the two can be compared with numbers. Only
the operating system and its HAL differ: same ArduPilot commit,
parameters, enabled features and compiler family.

| #    | Step                                  | Status      | Notes |
|------|---------------------------------------|-------------|-------|
| 4.1  | IcePilot Linux build                  | Not started | A Linux hwdef, `libraries/AP_HAL_Linux/hwdef/icepilot/`, with the FPGA driver written against `AP_HAL::SPIDevice` so both builds run the same driver |
| 4.2  | Linux test images                     | Not started | Raspberry Pi OS with its stock kernel and with a PREEMPT_RT kernel. **To decide:** one or both (both recommended) |
| 4.3  | Like-for-like builds                  | Not started | Same ArduPilot commit and parameters, the same features enabled (on QNX this needs 2.6), GCC 12 and `-O3` on both |
| 4.4  | Benchmark harness                     | Partial     | `examples/SchedulerTiming` measures the timer thread and `delay_microseconds()` on any HAL (2.19). To do: a script that collects the `PM` log messages and `@SYS/tasks.txt`, `timers.txt` and `threads.txt` over MAVLink FTP, repeats each run and produces one comparison table |
| 4.5  | CPU microbenchmarks                   | Not started | ArduPilot's `AP_Math` benchmarks (Google Benchmark), run on both systems |
| 4.6  | External timing                       | Not started | Main-loop jitter (minimum, average, 99th percentile, maximum) and IMU-to-motor latency. **To decide:** a logic analyzer, or timestamps from the IcePilot FPGA (part of 3.5) |
| 4.7  | Workloads                             | Not started | Idle, full logging, high-rate telemetry, scripting, a background CPU hog, and a network flood |
| 4.8  | Baseline results                      | Not started | Linux stock kernel, Linux PREEMPT_RT, and QNX as ported (2.1 to 2.16), before any optimization |
| 4.9  | Optimization results                  | Partial     | Each QNX optimization (2.19 to 2.23) measured on its own against the QNX baseline and against Linux. 2.19 against the QNX baseline: see 2.19; against Linux: to do (4.2) |
| 4.10 | Recovery and startup                  | Not started | Boot-to-ready time, and restart time after ArduPilot is killed (3.13) |

## Differences from the Linux port

Each of these follows from a real difference between QNX and Linux, and
is commented where it appears in the code. For steps not done yet, the
QNX column is the planned approach. Deliberate QNX optimizations of
`AP_HAL_QNX`, which go beyond what Linux does, are listed in table 2
from 2.19 onward.

| Area                     | Linux port                       | QNX port                                    | Reason |
|--------------------------|----------------------------------|---------------------------------------------|--------|
| C library                | glibc                            | Defines `_QNX_SOURCE`, links `libsocket`    | g++ defines `_GNU_SOURCE` on Linux but not `_QNX_SOURCE` on QNX, without which QNX hides many POSIX declarations. QNX keeps sockets in a separate library |
| Malloc wrapper           | Added before the configure checks | Added after the configure checks            | QNX has no symbol versioning, so `--wrap,malloc` also catches the `malloc` that `libc++.so` uses and the check programs fail to link |
| SITL linking             | Static                           | Dynamic                                     | The QNX SDP has no static `libsocket` |
| Directory entries        | System `struct dirent`           | ArduPilot's fixed-size `struct dirent`, filled from QNX's | QNX's `struct dirent` has a variable-length name and no `d_type` |
| Disk space               | `statfs()`                       | `statvfs()`                                 | QNX has no `statfs()` |
| Serial port setup        | `termios2` ioctls                | Standard `termios`, as on macOS and the BSDs | QNX has no `termios2`. Linux encodes standard baud rates as codes; QNX uses the numbers themselves, so standard `termios` sets any rate |
| Hardware flow control    | `CRTSCTS`                        | `IHFLOW \| OHFLOW`                          | QNX has separate input and output flags |
| Unbuffered serial writes (SITL) | `O_DIRECT \| O_SYNC`      | `O_SYNC`, as on OpenBSD                     | QNX has no `O_DIRECT` |
| Byte order functions     | ArduPilot's own                  | QNX's, from `net/netbyte.h`                 | QNX declares them as functions, which clash with ArduPilot's |
| lwIP `struct iovec`      | lwIP's own                       | QNX's                                       | QNX defines it in every file, through `sys/platform.h`; Linux only in `sys/uio.h`, which lwIP does not include |
| Event loop (`Poller`)    | `epoll`, `eventfd`, `timerfd`    | `poll()` or QNX pulses                      | Not available on QNX |
| Device callbacks         | A `timerfd` per callback, waited for with `epoll` (`PollerThread`) | One thread per bus that keeps each callback's next run time and sleeps until the earliest on one high-resolution timer (`DeviceBus`), as ChibiOS does | No `timerfd` or `epoll` on QNX. Each callback runs once per pass, so a slow one cannot starve the others |
| I2C                      | `i2c-dev`, one `I2C_RDWR` ioctl for several reads | `devctl()` to the QNX I2C driver, one per transfer; buses named `/dev/i2c<n>` | QNX I2C drivers take one write and read per message |
| I2C bus clock            | Set by the device tree           | The slowest device's, as on ChibiOS         | The QNX driver sets the clock for the whole bus |
| SPI                      | `spidev`, a write then a read as two messages of one ioctl | io-spi nodes named in the hwdef (`QNX_SPIDEV`); one exchange of both lengths | io-spi exchanges send and receive the same number of bytes, and QNX names nodes in the io-spi configuration |
| Missing SPI node         | Panics                           | The device is not created, so its driver reports it missing | A missing node is a board configuration problem that the driver can report |
| Buses                    | Closed with their last device    | Stay open once used                         | Devices that fail to probe come and go, while the bus thread may run callbacks |
| GPIO                     | `/dev/gpiomem` on the Raspberry Pi; no pin interrupts | Messages to `rpi_gpio`; pin interrupts from its pulses; outputs read as last written; only the board's GPIO pins | No `/dev/gpiomem` on QNX, and mapping the registers needs root. `rpi_gpio` cannot read outputs |
| PWM                      | sysfs, `/dev/mem`                | Through the IcePilot FPGA (3.7)             | No sysfs on QNX |
| RC input                 | `RCInput_RCProtocol` and board backends | `AP_RCProtocol`, as on ChibiOS          | Receivers on serial ports, without a board backend |
| CAN                      | SocketCAN                        | SITL: multicast CAN, as on macOS. HAL: off for now | No SocketCAN on QNX |
| Realtime scheduling      | Only when run as root            | For every user                              | QNX gives `SCHED_FIFO` to every user, up to priority 63 by default; ArduPilot uses up to 20 |
| CPU affinity             | `sched_setaffinity()`, `cpu_set_t`, any set of CPUs | Runmask, set with `ThreadCtl()`; one CPU or all CPUs | QNX has neither call nor type. QNX 8 also only accepts a runmask that matches a scheduling cluster, and by default these are each single CPU and all CPUs |
| Thread stack bounds      | `pthread_getattr_np()`           | Thread local storage (`__tls()`)            | QNX has no `pthread_getattr_np()`. Used only for stack-usage debugging |
| Semaphore timeouts       | `take()` polls every 200 µs; `BinarySemaphore` waits on `CLOCK_REALTIME` | Blocking `pthread_mutex_timedlock_monotonic()`; `CLOCK_MONOTONIC`; sub-tick timeouts | Faster hand-off, timeouts unaffected by setting the clock, and QNX kernel timeouts otherwise end on the 1 ms tick |
| Short sleeps             | `nanosleep()`, which Linux's high-resolution timers end on time | A QNX high-resolution timer per thread, waited for as a pulse | QNX's `nanosleep()` ends on the 1 ms clock tick. The QNX timers need the `high_resolution_timer` ability |
| Optional libraries       | LTTng tracing, `libiio`          | Not checked for                             | Both are Linux-only |

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

SITL for QNX builds the same way, with its own board:

```bash
source <path-to-sdp>/qnxsdp-env.sh
./waf configure --board SITL_aarch64_qnx
./waf copter
```

The executable is written to `build/SITL_aarch64_qnx/bin/arducopter`.

Simulation on hardware (2.24), the vehicle on the real QNX HAL with simulated
sensors, has a board too:

```bash
source <path-to-sdp>/qnxsdp-env.sh
./waf configure --board qnx-SimOnHardWare
./waf copter
```

The executable is written to `build/qnx-SimOnHardWare/bin/arducopter`. See
`libraries/AP_HAL_QNX/hwdef/qnx-SimOnHardWare/README.md` for how to run it.

## Running on a QNX target

On a target with SSH, such as the QNX quick start image for the
Raspberry Pi 4, copy the executable and run it as an ordinary user: QNX
gives it realtime priorities without root. For example, with MAVLink on
TCP port 5760:

```bash
scp build/icepilot/bin/arducopter <user>@<target-ip>:
ssh <user>@<target-ip> ./arducopter --serial0 tcp:0.0.0.0:5760
```

For sleeps shorter than the 1 ms clock tick, ArduPilot needs the
`high_resolution_timer` ability (see 2.19). Root has it; start it as
root and give another user only that ability with:

```bash
ssh <user>@<target-ip> sudo on -A nonroot,allow,high_resolution_timer -u <user> ./arducopter --serial0 tcp:0.0.0.0:5760
```

`./arducopter --help` lists the serial port options. Parameters are kept
in `/var/lib/ardupilot`, which must be writable by that user, for
example after `sudo mkdir -p /var/lib/ardupilot` and
`sudo chown <user> /var/lib/ardupilot`.

### Testing the buses and GPIO

Two examples check a board's buses and GPIO through the `AP_HAL`
interfaces, so they run on every HAL. Build them with
`./waf examples`, then run them on the target as above, with the
`high_resolution_timer` ability:

- `build/icepilot/examples/BusTest` scans each I2C bus with one-byte
  reads, as `i2cdetect -r` does; reads register 0 and the usual
  `WHO_AM_I` registers of each I2C device found and each SPI device of
  the board; and times periodic callbacks on the bus threads, which
  read `WHO_AM_I` of each SPI device and count the reads that differ.
  It never writes to a device.
- `build/icepilot/examples/GPIOTest` drives GPIO26, which must be free,
  and checks writes, reads and pin interrupts on its own edges. For
  another pin, configure with eg. `--define GPIO_TEST_PIN=6`.

Both need the fixed SPI and I2C drivers of the IcePilot boot image
(3.4).

### Starting through qconn

Otherwise, upload and start the executable through qconn (port 8000)
with the SDP debugger:

```bash
source <path-to-sdp>/qnxsdp-env.sh
ntoaarch64-gdb --batch \
  -ex "target qnx <target-ip>:8000" \
  -ex "file build/icepilot/bin/arducopter" \
  -ex "upload build/icepilot/bin/arducopter /tmp/arducopter" \
  -ex "set nto-cwd /tmp" \
  -ex run
```

SITL for QNX also needs Copter's default parameters and a vehicle model.
Upload the executable last, as the debugger runs the last file uploaded:

```bash
source <path-to-sdp>/qnxsdp-env.sh
ntoaarch64-gdb --batch \
  -ex "target qnx <target-ip>:8000" \
  -ex "file build/SITL_aarch64_qnx/bin/arducopter" \
  -ex "upload Tools/autotest/default_params/copter.parm /tmp/copter.parm" \
  -ex "upload build/SITL_aarch64_qnx/bin/arducopter /tmp/arducopter-sitl" \
  -ex "set nto-cwd /tmp" \
  -ex "set args --model quad --speedup 1 --defaults /tmp/copter.parm -I0" \
  -ex run
```

SITL waits for a ground station on TCP port 5760 before it starts
flying, for example `mavproxy.py --master tcp:<target-ip>:5760`. Add
`--wipe` to the arguments to start from clean parameters.

To stop SITL, interrupt the debugger (Ctrl+C) and let it exit. SITL
exits by itself when its parent, the QNX debug agent, goes away.

## License

The ArduPilot project is licensed under the GNU General Public
License, version 3 or (at your option) any later version
(SPDX identifier `GPL-3.0-or-later`).

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

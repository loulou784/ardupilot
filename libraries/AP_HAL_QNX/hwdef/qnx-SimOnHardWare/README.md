# QNX Simulation on Hardware

ArduPilot on `AP_HAL_QNX`, on a Raspberry Pi 4 running QNX SDP 8, with a
simulated vehicle, sensors and physics instead of autopilot hardware, as
the ChibiOS `*-SimOnHardWare` boards do. The whole vehicle runs on the real
QNX HAL: scheduler, semaphores, storage, filesystem and serial ports. It
needs nothing attached to the Raspberry Pi.

The simulated vehicle is a quadcopter, with the parameters of the ChibiOS
`CubeOrange-SimOnHardWare` board in `defaults.parm`.

## Building

Source the QNX SDP environment first, so that the `ntoaarch64` compilers are on
`PATH`:

```bash
source ~/qnx800/qnxsdp-env.sh
./waf configure --board qnx-SimOnHardWare
./waf copter
```

The binary is `build/qnx-SimOnHardWare/bin/arducopter`.

## Running

Run it on the Raspberry Pi with a ground station link, for example MAVLink on
TCP port 5760, and its parameters kept apart from those of a real board:

```bash
./arducopter --serial0 tcp:0.0.0.0:5760 -s /var/lib/ardupilot/sim
```

For sleeps shorter than the 1 ms clock tick, give it the
`high_resolution_timer` ability, as described in the top-level `README.md` of
the `qnx` branch.

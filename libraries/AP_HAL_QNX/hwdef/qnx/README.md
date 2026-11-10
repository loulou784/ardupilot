# QNX

Generic QNX configuration: ArduPilot on a Raspberry Pi 4 (aarch64) running
QNX SDP 8, with no autopilot hardware attached. It is the base for QNX boards
such as [IcePilot](../icepilot/README.md), and the target used to port and
benchmark `AP_HAL_QNX`.

## Building

Source the QNX SDP environment first, so that the `ntoaarch64` compilers are on
`PATH`:

```bash
source ~/qnx800/qnxsdp-env.sh
./waf configure --board qnx
./waf copter
```

The binary is `build/qnx/bin/arducopter`.

# IcePilot

IcePilot is an autopilot HAT for the Raspberry Pi 4, running QNX SDP 8. An
iCE40UP5K FPGA on the HAT handles PWM outputs, RC input and other I/O for the
Raspberry Pi.

The board is still being ported. See the QNX port status in the top-level
`README.md` of the `qnx` branch.

## Building

Source the QNX SDP environment first, so that the `ntoaarch64` compilers are on
`PATH`:

```bash
source ~/qnx800/qnxsdp-env.sh
./waf configure --board icepilot
./waf copter
```

The binary is `build/icepilot/bin/arducopter`.

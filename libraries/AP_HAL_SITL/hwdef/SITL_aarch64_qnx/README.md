# SITL_aarch64_qnx

Software-In-The-Loop target to run on AArch64 QNX, e.g. a Raspberry Pi 4 or a QEMU virtual machine running QNX SDP 8.0

Source the QNX SDP environment (`qnxsdp-env.sh`) before configuring, so that the `ntoaarch64` compilers are on the PATH.

Unlike the Linux SITL targets, the output binaries are dynamically linked, as the QNX SDP does not provide a static `libsocket`.

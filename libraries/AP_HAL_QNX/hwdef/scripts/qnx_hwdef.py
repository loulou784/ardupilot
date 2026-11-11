#!/usr/bin/env python3
'''
setup board.h for QNX

AP_FLAKE8_CLEAN

'''

import argparse
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class QNXHWDef(hwdef.HWDef):
    '''hwdef processing for QNX boards, following LinuxHWDef.  SPI devices
    are QNX_SPIDEV lines, as LINUX_SPIDEV lines on Linux, with one more
    column: the io-spi device node, which QNX names in the io-spi
    configuration, where Linux has /dev/spidev<bus>.<subdev>:

    QNX_SPIDEV name bus subdev node mode bits_per_word cs_pin lowspeed highspeed
    QNX_SPIDEV "icm42688" 0 0 "/dev/io-spi/spi0/imu1" SPI_MODE_3 8 SPI_CS_KERNEL 1*MHZ 10*MHZ
    '''

    def __init__(self, **kwargs):
        super(QNXHWDef, self).__init__(**kwargs)
        # a list of QNX_SPIDEV devices
        self.qnx_spidev = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write(self.define_line(d))

        self.write_SPI_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        a = self.split_line(line, posix=False)
        if a[0] == 'QNX_SPIDEV':
            self.process_line_qnx_spidev(line, depth, a)

        super(QNXHWDef, self).process_line(line, depth, a)

    def process_line_undef(self, line, depth, a):
        for u in a[1:]:
            for dev in self.qnx_spidev:
                if u == dev[0]:
                    self.qnx_spidev.remove(dev)

        super(QNXHWDef, self).process_line_undef(line, depth, a)

    def process_line_qnx_spidev(self, line, depth, a):
        self.qnx_spidev.append(a[1:])

    def write_SPI_device_table(self, f):
        '''write SPI device table'''

        devlist = []
        for dev in self.qnx_spidev:
            if len(dev) != 9:
                self.error(f"Badly formed QNX_SPIDEV line {dev} {len(dev)=}")
            (name, bus, subdev, node, mode, bits_per_word, cs_pin, lowspeed, highspeed) = dev
            if not (node.startswith('"/dev/') and node.endswith('"')):
                self.error(f"Bad node {node} in QNX_SPIDEV line {dev}: expected a quoted /dev path")
            if mode not in ['SPI_MODE_0', 'SPI_MODE_1', 'SPI_MODE_2', 'SPI_MODE_3']:
                self.error(f"Bad mode {mode} in QNX_SPIDEV line {dev}")
            if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
                self.error(f"Bad lowspeed value {lowspeed} in QNX_SPIDEV line {dev}")
            if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
                self.error(f"Bad highspeed value {highspeed} in QNX_SPIDEV line {dev}")
            f.write(
                f'#define HAL_SPI_DEVICE{len(devlist)+1} SPIDesc({name:12s}, {bus}, {subdev}, {node}, {mode}, {bits_per_word}, {cs_pin}, {lowspeed:6s}, {highspeed:6s})\n'  # noqa
            )
            devlist.append('HAL_SPI_DEVICE%u' % (len(devlist)+1))

        self.write_device_table(f, "spi devices", "HAL_SPI_DEVICE_LIST", devlist)

    def write_SPI_config(self, f):
        '''write SPI config defines'''

        self.write_SPI_device_table(f)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("qnx_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = QNXHWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()

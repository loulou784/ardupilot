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
    '''hwdef processing for QNX boards, following LinuxHWDef.  There is no
    SPI device table yet: it arrives with the SPI driver, as Linux's
    LINUX_SPIDEV lines describe spidev devices that QNX does not have'''

    def __init__(self, **kwargs):
        super(QNXHWDef, self).__init__(**kwargs)

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write(self.define_line(d))

        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        super(QNXHWDef, self).process_line(line, depth)


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

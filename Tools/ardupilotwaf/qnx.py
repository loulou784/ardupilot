# encoding: utf-8

"""
Waf tool for QNX build

AP_FLAKE8_CLEAN
"""

import os
import sys
import traceback

import boards

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_QNX/hwdef/scripts'))
import qnx_hwdef  # noqa: E402


def configure(cfg):

    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    env.SRCROOT = cfg.srcnode.make_node('').abspath()
    env.BUILDROOT = bldnode.make_node('').abspath()

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)


def generate_hwdef_h(env):
    '''run qnx_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_QNX/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = qnx_hwdef.QNXHWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        build_option_defines=boards.build_option_defines(env.OPTIONS),
        quiet=False,
    )
    hwdef_obj.run()

    return hwdef_obj

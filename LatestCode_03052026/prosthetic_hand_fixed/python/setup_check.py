"""setup_check.py — verify both boards are alive and 4 sensors enumerate.

Usage:
    python setup_check.py
        Auto-detects ports, pings each board, asks XIAO for enum.
        Exit code 0 on full success, 1 on any failure.

If the XIAO reports fewer than 4 sensors we issue a recovery 'r'
command and re-check once. We deliberately do not loop forever — if
the strip has a hardware fault, the user needs to see the failure.
"""

from __future__ import annotations

import sys
import time

from hand import constants as C
from hand.controller import HandController, detect_boards


def main() -> int:
    print('[setup_check] detecting boards...')
    xiao_port, openrb_port = detect_boards()
    if xiao_port is None:
        print('[setup_check] FAIL: XIAO ESP32S3 not found')
        return 1
    if openrb_port is None:
        print('[setup_check] FAIL: OpenRB-150 not found')
        return 1
    print(f'[setup_check]   XIAO   on {xiao_port}')
    print(f'[setup_check]   OpenRB on {openrb_port}')

    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl:
        xiao_ok, openrb_ok = ctrl.ping_both()
        if not xiao_ok:
            print('[setup_check] FAIL: XIAO did not respond to ping')
            return 1
        if not openrb_ok:
            print('[setup_check] FAIL: OpenRB did not respond to ping')
            return 1
        print('[setup_check] both boards pong OK')

        enum = ctrl.enum_xiao()
        if enum is None or not enum.args:
            print('[setup_check] FAIL: XIAO did not report enumeration')
            return 1
        n_found = int(enum.args[0])
        found_addrs = enum.args[1:]
        print(f'[setup_check] XIAO enum: {n_found}/4 found  ({", ".join(found_addrs) or "none"})')

        if n_found < C.NUM_SENSORS:
            print('[setup_check] requesting XIAO recovery...')
            ctrl.send_xiao('r')
            ready = ctrl.wait_for_event('xiao', 'ENUM_READY', 5.0)
            if ready is None:
                fail = ctrl.wait_for_event('xiao', 'ENUM_FAIL', 0.2)
                if fail is not None:
                    print(f'[setup_check] FAIL: XIAO recovery exhausted ({" ".join(fail.args)})')
                else:
                    print('[setup_check] FAIL: no ENUM_READY after recovery')
                return 1
            # Re-poll the latest enum line for the success count.
            enum2 = ctrl.enum_xiao()
            if enum2 is None or int(enum2.args[0]) < C.NUM_SENSORS:
                print('[setup_check] FAIL: still missing sensors after recovery')
                return 1
            print('[setup_check] XIAO recovery OK: all 4 sensors present')

        motor = ctrl.enum_openrb()
        if motor is None:
            print('[setup_check] FAIL: OpenRB motor enumeration timed out')
            return 1
        print(f'[setup_check] OpenRB motor: {" ".join(motor.args)}')

    print('[setup_check] ALL CHECKS PASSED.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

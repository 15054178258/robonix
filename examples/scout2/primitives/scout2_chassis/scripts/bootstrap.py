#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Bootstrap entry for scout2_chassis primitive — adds conda ROS2 paths + driver dir."""
from __future__ import annotations

import os
import sys

# Re-add conda env's default paths that PYTHONPATH env var would strip
for _p in (
    '/opt/ros/humble/lib/python3.10/site-packages',
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/home/szh/miniforge3/envs/scout2-py/lib/python310.zip',
    '/home/szh/miniforge3/envs/scout2-py/lib/python3.10',
    '/home/szh/miniforge3/envs/scout2-py/lib/python3.10/lib-dynload',
    '/home/szh/miniforge3/envs/scout2-py/lib/python3.10/site-packages',
):
    if _p and _p not in sys.path:
        sys.path.insert(0, _p)
# Prepend: driver parent dir (host_pkg) + proto_gen
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _parent)
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/proto_gen'))

from chassis_driver import driver as _driver  # noqa: E402

if __name__ == '__main__':
    _driver.scout2_chassis.run()

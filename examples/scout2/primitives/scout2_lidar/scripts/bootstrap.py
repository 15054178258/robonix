#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Bootstrap entry for scout2_lidar primitive — adds conda ROS2 paths + driver dir."""
from __future__ import annotations

import os
import sys

for _p in (
    '/opt/ros/humble/lib/python3.10/site-packages',
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/home/user/miniforge3/envs/robonix-scout2/lib/python310.zip',
    '/home/user/miniforge3/envs/robonix-scout2/lib/python3.10',
    '/home/user/miniforge3/envs/robonix-scout2/lib/python3.10/lib-dynload',
    '/home/user/miniforge3/envs/robonix-scout2/lib/python3.10/site-packages',
):
    if _p:
        sys.path.insert(0, _p)
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _parent)
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/proto_gen'))
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/robonix_mcp_types'))
# Robonix shared library — needed by all primitives
sys.path.insert(0, '/home/user/robonix/pylib/robonix-api')

from lidar_driver import driver as _driver  # noqa: E402

if __name__ == '__main__':
    _driver.scout2_lidar.run()

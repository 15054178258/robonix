#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Bootstrap entry for scout2_camera primitive — adds conda ROS2 paths + driver dir."""
from __future__ import annotations

import os
import sys

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
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _parent)
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/proto_gen'))
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/robonix_mcp_types'))
sys.path.insert(0, os.path.join(_parent, 'robonix_mcp_types'))

from camera_driver import driver as _driver  # noqa: E402

if __name__ == '__main__':
    _driver.scout2_camera.run()

# Third-party software and redistribution notices

The repository's original integration code is Apache-2.0. The following
pinned upstream components retain their own licenses. This list is an
inventory, not legal advice; distributors remain responsible for satisfying
the applicable terms.

| Component | Use | License | Authoritative text |
|---|---|---|---|
| Unitree SDK2 | Statically linked into the isolated chassis and camera daemons | BSD-3-Clause | `third_party/unitree_sdk2/LICENSE` |
| Unitree ROS 2 | Go2 ROS message definitions | BSD-3-Clause | `third_party/unitree_ros2/LICENSE` |
| Eclipse CycloneDDS | Private `libddsc` runtime shipped beside the SDK daemons | EPL-2.0 OR EDL-1.0 | `third_party/unitree_sdk2/licenses/eclipse-cyclonedds/cyclonedds/LICENSE` |
| Eclipse CycloneDDS-CXX | Private `libddscxx` runtime shipped beside the SDK daemons | EPL-2.0 OR EDL-1.0 | `third_party/unitree_sdk2/licenses/eclipse-cyclonedds/cyclonedds-cxx/LICENSE` |
| Eclipse Iceoryx | Unitree SDK2 dependency | Apache-2.0 | `third_party/unitree_sdk2/licenses/eclipse-iceoryx/iceoryx/LICENSE` |
| RapidJSON and its listed third-party components | Unitree SDK2 dependency | MIT and the notices in the upstream license file | `third_party/unitree_sdk2/licenses/Tencent/rapidjson/LICENSE` |
| Unitree Go2 URDF/DAE assets | Robot model and TF geometry | BSD-3-Clause | `packages/go2_description/LICENSE.unitree` |

Both SDK-daemon CMake install rules copy these SDK/runtime license texts into
`share/licenses/unitree_sdk2/` beside the installed image. Do not distribute a
detached daemon binary or private DDS libraries without their corresponding
license directory and this repository's `NOTICE`.

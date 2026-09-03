---
description: Publishes the pinned Unitree Go2 body model and fixed TF tree; it has no motion or posture interface.
---

# Go2 robot description

This primitive obtains the deployment URDF from Soma, verifies that it is
byte-for-byte the pinned `urdf/go2_robonix.urdf`, and starts the standard ROS 2
`robot_state_publisher` in the deployment's CycloneDDS graph.

It publishes fixed joints on `/tf_static` and consumes ordinary joint states
for movable joints. It never opens Unitree SDK2 and exposes only the standard
Robonix robot-description lifecycle driver.

# Known limitations before physical commissioning

- Scene, Mapping and Navigation require the compatibility changes checked by
  `scripts/verify_upstream_compatibility.sh`. Until the corresponding upstream
  PRs merge and the remote package caches are updated, build/start deliberately
  fail instead of launching incompatible ROS contracts or RMWs. Each successful
  build records the exact revisions in ignored `rbnx-build/upstream-lock.txt`.
- Robonix must also include the audited provider bind-host change; otherwise
  the compatibility gate rejects boot rather than exposing capability servers.
  Scene's debug UI and Mapping's administration UI are intentionally absent;
  use the read-only dashboard or a local visualization-only RViz session.
- The public vending-machine pose is intentionally unverified and cannot
  trigger navigation.
- Go2 model sensor frames come from the official public model but must be
  compared with the exact hardware edition.
- Unitree VideoClient exposes an RGB JPEG stream. The base integration does
  not invent an RGB-D depth image; obstacle avoidance uses the onboard lidar.
- CameraInfo deliberately reports `K[0] == 0` until the physical camera is
  calibrated. The deployment does not advertise a Robonix intrinsics or
  extrinsics contract before real measured values are available.
- Nav2 footprint and limits are conservative provisional values.
- PointCloud2 per-point timestamps are not yet verified on the target firmware,
  so Mapping and scan-projection deskew are disabled by default.
- With external chassis odometry, the audited Mapping revision does not also
  subscribe to IMU directly; validated SportModeState orientation reaches it
  through odometry instead. This profile does not claim direct RTAB-Map IMU
  fusion.
- The first version localizes a known semantic landmark. It does not search
  camera frames for an unknown vending machine.
- A VLM/LLM endpoint is still required by Robonix Pilot for natural-language
  tool selection; its credentials are always local secrets.

# Supervised long-distance voice navigation validation

Date: 2026-07-31 (Asia/Shanghai)

## Result

The supervised Unitree Go2 reference deployment completed the full operator
flow:

1. the local Robonix Client accepted a Chinese voice request;
2. the deterministic semantic route resolved a verified long-distance landmark;
3. Nav2 planned and executed the physical outbound route on the generation-1
   map;
4. the Client then dispatched the verified map-start landmark and the robot
   completed the autonomous return;
5. the operator accepted the small placement deviation as non-blocking and
   retained video of the successful round trip.

The physical run was supervised with the remote in hand and the corridor
cleared. Stop, cancel, fresh-state checks, single-controller ownership and the
independent chassis watchdog remained active. The daemon returned to DISARMED
after the run.

## Full-stack surfaces

The validated loopback deployment exposes:

- Client on port 7860;
- Scene map and goal selection on port 50107;
- Mapping on port 8091;
- the Go2 sensor/navigation dashboard on port 8092.

The generation-1 map, operator initial-pose sidecar and site-specific semantic
landmarks are intentionally not committed. They remain deployment data rather
than reusable source configuration.

## Avoidance boundary

Both Nav2 costmaps use the live MID-360-derived `LaserScan` through obstacle and
inflation layers with marking and clearing enabled. The D435i stream remains a
read-only preview and is not a costmap source. Because the corridor was cleared,
this run validates the active laser/costmap chain but is not a dedicated
placed-obstacle avoidance acceptance test.

No raw audio, logs, map files, screenshots, recordings, credentials or
device-specific calibration are included in this report.

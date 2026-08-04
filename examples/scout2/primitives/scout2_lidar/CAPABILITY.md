# Scout2 Lidar (`robonix/primitive/lidar`)

Scout2's 2D planar lidar + optional 3D point cloud. Supplementary distance sensor for collision avoidance.

## Tools

### `snapshot` — `robonix/primitive/lidar/snapshot`
- input: none
- returns: `sensor_msgs/LaserScan` JSON. `ranges[i]` is the distance (meters)
  at angle `angle_min + i*angle_increment`.
- use cases:
  - "is there an obstacle within X m in front of me?"  →  scan the middle
    of `ranges[]` for the smallest value.
  - "where is the nearest open space?"  →  argmax of `ranges[]`.
- DO NOT use lidar to localize on a map; it has no map context.

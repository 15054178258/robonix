# Go2 高密度二维建图离线检查点

- 时间：2026-07-24 02:13:58 CST（UTC+08:00）
- 状态：`OFFLINE CHECKPOINT`
- 边界：代码、离线数据和容器启动已验收；新的实机地图质量尚待机器人恢复连接后验证

## 已完成

1. 建图输入从单帧稀疏 MID-360 点云改为显式启用的高密度二维链：

   ```text
   /scanner/cloud
     -> lidar_deskewing
     -> 4 帧 odom 对齐组云
     -> 0.5° / 720 槽 LaserScan
     -> /rtabmap/scan_dense
     -> RTAB-Map
   ```

2. 新增严格的相邻边 ICP 精修候选：

   ```text
   Reg/Strategy=1
   Reg/Force3DoF=true
   RGBD/NeighborLinkRefining=true
   RGBD/ProximityBySpace=false
   RGBD/ProximityPathMaxNeighbors=0
   Icp/PointToPlane=true
   Icp/PointToPlaneK=5
   Icp/PointToPlaneMinComplexity=0.02
   Icp/PointToPlaneLowComplexityStrategy=1
   Icp/CorrespondenceRatio=0.20
   Icp/MaxCorrespondenceDistance=0.15
   Icp/MaxTranslation=0.10
   Icp/MaxRotation=0.10
   Icp/RangeMin=0.492
   Icp/RangeMax=6.0
   ```

   空间近邻回环保持关闭，避免相似走廊被错误连接。该功能默认关闭，仅由
   `dense_scan_refine_neighbors: true` 显式启用。

3. 固化下一次实机使用的配置：

   `config/mapping_dense2d_refined_nomotion.json`

4. 修复无运动时间中继对一次有界 callback 抖动直接整体退出的问题。触发样本会被丢弃，
   corrected 输出暂停，所有核心流各取得两个新鲜样本后恢复；运动配置和真实时间回退、
   陈旧数据、超出 hard ceiling 的故障处理均未放宽。

## 离线 A/B

证据目录：

`logs/go2-readonly/mapping-offline-neighbor-ab-20260724T015515CST/`

- 较宽的保守参数会过度校正，不采用。
- 严格结果包含 1052 条有向相邻 Link；4 条平移修正超过 5 cm，对应 2 个独立相邻边。
- 严格结果没有 yaw 修正超过 3°，最大 yaw 修正为 1.473°。
- 两组均没有形成回环闭合，不能据此宣称实机走廊漂移已经解决。

## 验证

- 最新建图配置：10/10
- NeighborLinkRefining 严格参数定向测试：6/6
- `service-map-rbnx` 完整测试：56/56
- affine discipline 与 workstation stamp layer：120/120
- time-sync 与仓库安全：59/59
- JSON、Bash、Python AST/编译、`git diff --check`：通过

在现有 `robonix-mapping` 镜像中完成 30 秒回环 DDS 启动冒烟。实际日志确认：

- 去畸变、四帧组云、二维投影、RTAB-Map、位姿转发五个节点均启动；
- RTAB-Map 订阅 `/rtabmap/scan_dense`，未订阅原始 `scan_cloud`；
- 上述严格 ICP 参数及空间近邻关闭项均实际落地；
- 仅出现机器人断连时预期的“无传感器数据”告警；
- 临时容器和临时数据库已在测试结束时清理。

## 恢复连接后直接续接

1. 只读确认 cloud、odom、TF 和 `/rtabmap/scan_dense` 已恢复且时间戳连续。
2. 使用最新配置启动全新 Mapping，不复用已发生漂移的旧活动数据库。
3. 提醒操作员：**现在开始录屏**。
4. 使用官方遥控器低速人工采集；Robonix 运动输出保持禁用。
5. 本轮建图期间不点击“锁定”、不搬运机器人。
6. 首轮路线：静止 10 秒、直行约 3 米、缓慢转 90°、再直行约 3 米、
   静止 10 秒；场地安全时原路返回。
7. 保存地图、UI 截图、日志和数据库封闭副本，检查墙线连续性、射线、直角、
   错误接路、yaw 修正和 TF 连续性。

只有新图质量通过后，才进入保存地图后的重定位和后续导航验收。

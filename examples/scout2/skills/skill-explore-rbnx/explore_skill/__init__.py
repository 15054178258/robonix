# SPDX-License-Identifier: MulanPSL-2.0
"""explore_skill — frontier-based autonomous exploration skill.

Modules:
  - atlas_bridge: entrypoint. Registers cap with atlas, resolves
    map + nav contracts, runs the FastMCP server with 3 tools.
  - frontier: WFD frontier extraction + DBSCAN-style clustering +
    info_gain/travel_cost scoring on nav_msgs/OccupancyGrid.
  - controller: per-task state machine. IDLE → EXPLORING → terminal.
    Loops: pick frontier → call nav/navigate → poll nav/status → repeat.
  - mcp_tools: FastMCP @tool decorators wrapping the controller's
    public API; declared with atlas via the package's
    capabilities/{explore,status,cancel}.v1.toml.
"""

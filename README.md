# 🤖 ROS2 Autonomous Navigation (TurtleBot3)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Nav2](https://img.shields.io/badge/Nav2-Stable-brightgreen)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python](https://img.shields.io/badge/Python-3.12-yellow)

---

## 📋 Overview

Comparative benchmark of SLAM Toolbox and Cartographer integrated with Nav2 for autonomous navigation on TurtleBot3 Waffle in ROS2 Jazzy. Conducted systematic parameter tuning experiments documenting narrow-passage navigation challenges, discovering the "Letterbox Trap" phenomenon where geometric precision conflicted with navigational safety, and implemented an architectural waypoint decomposition solution achieving 100% success rate.

---

## ✨ Features

**Dual SLAM Implementation**
- SLAM Toolbox (online_async configuration)
- Cartographer

**Nav2 Navigation**
- Single-goal navigation via RViz
- Waypoint following using Simple Commander API
- AMCL localization on pre-generated maps

**Data Collection**
- rosbag2 recordings (MCAP format)
- RViz screenshots and screen recordings
- Foxglove Studio visualization and logging

**Systematic Parameter Tuning**
- 7 sequential experiments documented
- Inflation radius, DWB critics, AMCL precision
- Robot geometry (radius vs. footprint)
- Discovery of safety thresholds and geometric paradoxes

---

## 🖼️ Visual Proof

### Mapping Phase

<table>
<tr>
<td width="50%">

#### Gazebo House Environment
![Gazebo House](results/screenshots/Slam_toolbox/gazebo_house.png)
*Simulated residential environment with narrow doorway constraint*

</td>
<td width="50%">

#### SLAM Toolbox Mapping
![SLAM Mapping](results/screenshots/Slam_toolbox/rviz_slam_map.png)
*Real-time SLAM map generation in RViz*

</td>
</tr>
</table>

### Navigation Phase

<table>
<tr>
<td width="50%">

#### Nav2 Path Planning
![Path Planning](results/screenshots/Slam_toolbox/nav2_path_costmap.png)
*Global and local costmaps with planned path*

</td>
<td width="50%">

#### Waypoint Navigation
![Waypoints](results/screenshots/Slam_toolbox/waypoints_rviz.png)
*Multi-waypoint mission execution*

</td>
</tr>
</table>

<table>
<tr>
<td width="50%">

#### Cartographer Navigation
![Cartographer Nav](results/screenshots/Cartographer/nav2_goal_cartographer.png)
*Nav2 goal navigation on Cartographer-generated map*

</td>
<td width="50%">

#### Foxglove Dashboard
![Foxglove](results/screenshots/Cartographer/foxglove_dashboard.png)
*3D map, TF tree, velocity plots, pose data, logs*

</td>
</tr>
</table>

### Video Demonstrations

| Demo | Description | Content |
|------|-------------|---------|
| [Nav2 RViz Goals](results/videos/Slam_toolbox/nav2_rviz_goals.mp4) | Single-goal navigation | SLAM Toolbox map with live planning |
| [Waypoint Following](results/videos/Slam_toolbox/simple_commander_waypoints.mp4) | Programmatic navigation | Simple Commander API execution |
| [Foxglove Monitoring](results/videos/Cartographer/simple_commander_foxglove.mp4) | Telemetry dashboard | Cartographer map with live metrics |

---

## 🏗️ Architecture

```
                   ┌─────────────────────────────────────────────────────────────┐
                   │                     Gazebo Simulation                       │
                   │                  (turtlebot3_house.world)                   │
                   └───────────────────────────┬─────────────────────────────────┘
                                               │
                                   ┌───────────┴───────────┐
                                   │                       │
                            ┌──────▼──────┐         ┌─────▼──────┐
                            │    SLAM     │         │    Nav2    │
                            │   Backend   │         │   Stack    │
                            │ SLAM Toolbox│         │  + AMCL    │
                            │ Cartographer│         │            │
                            └──────┬──────┘         └─────┬──────┘
                                   │                      │
                                   │    ┌─────────────────┘
                                   │    │
                            ┌──────▼────▼──────┐
                            │   Visualization  │
                            │   - RViz2        │
                            │   - Foxglove     │
                            └──────────────────┘
                                    │
                             ┌──────▼──────┐
                             │   rosbag2   │
                             │    (MCAP)   │
                             └─────────────┘
```

**Data Flow:**
1. Mapping: Gazebo → SLAM Backend → Map (PGM/YAML)
2. Navigation: Map + Goal → Nav2 Planner → Controller → /cmd_vel → Gazebo
3. Monitoring: Topics → RViz/Foxglove + rosbag2 recording

---

## 📊 Results Summary

| SLAM Backend | Map Quality | Nav2 Integration | Doorway Handling | Recorded Runs |
|--------------|-------------|------------------|------------------|---------------|
| SLAM Toolbox | Complete | Stable | Requires waypoint strategy | 2 rosbags |
| Cartographer | Complete | Stable | Requires waypoint strategy | 2 rosbags |

**Qualitative Observations:**
- Both backends produced navigable maps
- Nav2 path planning succeeded in open areas
- Narrow doorway (0.81m) triggered recovery behaviors before aborting
- Waypoint decomposition strategy achieved 100% success rate
- AMCL localization remained stable throughout missions

---

## ⚠️ Engineering Challenges

### Narrow Doorway Navigation & The Letterbox Trap

**Physical Constraint:** 0.81m doorway width with adjacent letterbox obstacle vs. TurtleBot3 Waffle (306mm width) + inflation

**Systematic Testing** (7 experiments documented in [`docs/tuning.md`](docs/tuning.md)):

| Test | Key Parameters | Outcome | Discovery |
|------|----------------|---------|-----------|
| 01–04 | Inflation, DWB critics, AMCL tuning | Failed | Parameter tuning hit diminishing returns |
| 05 | `robot_radius: 0.13` | Collision | Values < 0.15m unsafe for Waffle |
| 06 | Rectangular footprint `[0.21, 0.165]` | "Letterbox Trap" | Precision invited unsafe gap planning |
| 07 | `robot_radius: 0.15` + Waypoint Decomposition | 100% success | Architectural solution over parameter tuning |

**The Letterbox Trap Discovery:**

When using an accurate rectangular footprint (Test 06), the Nav2 global planner identified a "theoretically valid" path through a ~0.2m gap between a letterbox and the wall. The robot confidently approached this gap, detected collision risk at the threshold, and entered an abort loop after approaching within 2cm of the obstacle.

> **Key Insight:**
> 
> ### Geometric precision ≠ Navigational safety

An accurate footprint representation invites the planner to explore every theoretically possible gap, including those unsafe for:
- Sensor coverage limitations
- Wheel slippage near obstacles  
- Odometry drift during tight maneuvers

**Final Solution: Waypoint Decomposition Strategy**

Rather than aggressive parameter tuning (which degraded open-area navigation), implemented a mission-level architectural approach:

Conservative Parameters:
```yaml
robot_radius: 0.15        # Circular buffer excludes tiny gaps
inflation_radius: 0.28
BaseObstacle.scale: 0.08  # Higher safety margin
```

Staged Waypoint Navigation:
```python
waypoints = [
    (4.5, 5.3, 1.57),  # WP1: Align before doorway
    (3.0, 2.5, 1.57),  # WP2: Interior room goal
    (4.5, 5.3, 1.57),  # WP3: Exit alignment  
    (0.3, 3.0, 0.0)    # WP4: Return to origin
]
```

Results:
- 4/4 waypoints reached (100% success rate)
- 0 collisions during doorway traversal
- Baseline parameters preserved (safe for all environments)
- Repeatable across multiple runs

Safety Thresholds Identified:
- `robot_radius < 0.15m` → Physical collisions (Test 05)
- Rectangular footprint → Letterbox trap (Test 06)
- Aggressive critic weights → Degrades open-area navigation

Complete experimental log with parameter diffs and architectural rationale: [`docs/tuning.md`](docs/tuning.md)

---

## 🚀 Quickstart

Build workspace:
```bash
cd tb3-nav2-slam-benchmark
colcon build --symlink-install
source install/setup.bash
```

Launch Gazebo + TurtleBot3:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Launch SLAM Toolbox (new terminal):
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=./config/slam_toolbox/online_async_stable.yaml
```

Launch Nav2 on saved map (new terminal):
```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=./results/maps/house_slam_toolbox_draft.yaml
```

Open RViz (new terminal):
```bash
ros2 run rviz2 rviz2 -d ./config/rviz/slam_mapping.rviz
```

Record mission (optional):
```bash
ros2 bag record -o mission_run /tf /cmd_vel /scan /amcl_pose
```

See [`docs/runs.md`](docs/runs.md) for complete command reference.

---

## 📁 Repository Structure

```
tb3-nav2-slam-benchmark/
├── config/                  # ROS2 config files
│   ├── slam_toolbox/        # SLAM params
│   ├── cartographer/        # Cartographer params
│   ├── nav2/                # Nav2 tuning experiments
│   └── rviz/                # RViz configs
├── docs/                    # Documentation
│   ├── setup.md             # Environment setup
│   ├── runs.md              # Run commands
│   ├── tuning.md            # 7-test experimental log
│   ├── troubleshooting.md
│   └── metrics.md           # Evaluation templates
├── results/
│   ├── maps/                # PGM + YAML maps
│   ├── rosbags/             # MCAP recordings
│   ├── screenshots/         # RViz + Foxglove
│   └── videos/              # Screen recordings
├── scripts/                 # Automation scripts
└── waypoint_following/      # Simple Commander API
```

---

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy Jalisco |
| Robot | TurtleBot3 Waffle |
| SLAM | SLAM Toolbox, Cartographer |
| Navigation | Nav2, AMCL |
| Visualization | RViz2, Foxglove Studio |
| Recording | rosbag2 (MCAP) |

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [Setup Guide](docs/setup.md) | Dependencies, workspace build |
| [Run Commands](docs/runs.md) | Copy-paste launch commands |
| [Tuning Log](docs/tuning.md) | 7-test experimental log with Letterbox Trap analysis |
| [Troubleshooting](docs/troubleshooting.md) | Common issues |
| [Metrics Template](docs/metrics.md) | Evaluation framework |

---

## 🔄 Reproducibility

All maps, rosbags, and configuration files are version-controlled. See [`scripts/quickstart.sh`](scripts/quickstart.sh) for automated demo.

---

## 📜 License
Apache-2.0

---

## 🙏 Acknowledgments

- TurtleBot3 navigation stack: ROBOTIS
- Nav2 framework: Open Navigation LLC
- SLAM implementations: Steve Macenski (SLAM Toolbox), Google (Cartographer)

---

<div align="center">

Star this repo if you find it useful

[Open an Issue](../../issues) · [Submit a PR](../../pulls)

</div>
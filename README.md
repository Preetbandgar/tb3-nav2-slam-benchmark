# 🤖 ROS2 Autonomous Navigation (Turtlebot3)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Nav2](https://img.shields.io/badge/Nav2-Stable-brightgreen)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python](https://img.shields.io/badge/Python-3.12-yellow)

---

## 📋 Overview

### 🎯 What
ROS2 Jazzy benchmark comparing two SLAM backends (SLAM Toolbox, Cartographer) integrated with Nav2 for autonomous navigation on a TurtleBot3 waffle in a simulated residential environment.

### 🔍 Why
Evaluate practical navigation performance, understand SLAM-Nav2 integration workflows, and document tuning challenges encountered with narrow doorways (0.81m width).

### ⚙️ How
Generated maps using both SLAM systems, executed single-goal and waypoint-based navigation missions, recorded rosbags (MCAP), visualized in RViz and Foxglove Studio, documented tuning attempts and pragmatic workarounds.

---

## ✨ Features

### 🗺️ Dual SLAM Implementation
- SLAM Toolbox (online_async configuration)
- Cartographer

### 🧭 Nav2 Navigation
- Single-goal navigation via RViz
- Waypoint following using Simple Commander API
- AMCL localization on pre-generated maps

### 💾 Data Collection
- rosbag2 recordings (MCAP format)
- RViz screenshots and screen recordings
- Foxglove Studio visualization and logging

### 🔧 Documented Tuning Process
- Inflation radius adjustments
- Footprint/radius experimentation
- Controller tolerance tuning
- Recovery behavior configuration
- Baseline parameter restoration

---

## 🖼️ Visual Proof

### 🏗️ Mapping Phase

<table>
<tr>
<td width="50%">

#### 🏠 Gazebo House Environment
![Gazebo House](results/screenshots/Slam_toolbox/gazebo_house.png)
*Simulated residential environment with narrow doorway constraint*

</td>
<td width="50%">

#### 🗺️ SLAM Toolbox Mapping
![SLAM Mapping](results/screenshots/Slam_toolbox/rviz_slam_map.png)
*Real-time SLAM map generation in RViz*

</td>
</tr>
</table>

### 🚀 Navigation Phase

<table>
<tr>
<td width="50%">

#### 📈 Nav2 Path Planning
![Path Planning](results/screenshots/Slam_toolbox/nav2_path_costmap.png)
*Global and local costmaps with planned path*

</td>
<td width="50%">

#### 📍 Waypoint Navigation
![Waypoints](results/screenshots/Slam_toolbox/waypoints_rviz.png)
*Multi-waypoint mission execution*

</td>
</tr>
</table>

<table>
<tr>
<td width="50%">

#### 🧭 Cartographer Navigation
![Cartographer Nav](results/screenshots/Cartographer/nav2_goal_cartographer.png)
*Nav2 goal navigation on Cartographer-generated map*

</td>
<td width="50%">

#### 📊 Foxglove Dashboard
![Foxglove](results/screenshots/Cartographer/foxglove_dashboard.png)
*3D map, TF tree, velocity plots, pose data, logs*

</td>
</tr>
</table>

### 🎬 Video Demonstrations

| 📹 Demo | 📝 Description | ⏱️ Content |
|---------|---------------|-----------|
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

**🔄 Data Flow:**
1. **Mapping:** Gazebo → SLAM Backend → Map (PGM/YAML)
2. **Navigation:** Map + Goal → Nav2 Planner → Controller → /cmd_vel → Gazebo
3. **Monitoring:** Topics → RViz/Foxglove + rosbag2 recording

---

## 📊 Results Summary

| 🗺️ SLAM Backend | 🎯 Map Quality | 🔗 Nav2 Integration | 🚪 Doorway Handling | 📦 Recorded Runs |
|-----------------|----------------|---------------------|---------------------|------------------|
| **SLAM Toolbox** | ✅ Complete | ✅ Stable | ⚠️ Requires workaround | 2 rosbags |
| **Cartographer** | ✅ Complete | ✅ Stable | ⚠️ Requires workaround | 2 rosbags |

**📝 Qualitative Observations:**
- ✅ Both backends produced navigable maps
- ✅ Nav2 path planning succeeded in open areas
- ⚠️ Narrow doorway (~0.81m) triggered recovery behaviors (rotate, backup) before aborting
- ✅ Two-step waypoint navigation (entry point → final goal) successfully navigated doorway
- ✅ AMCL localization remained stable throughout missions

---

## ⚠️ Known Limitations

### 🚪 Narrow Doorway Constraint

**🔍 Physical Constraint:** 0.81m doorway width vs. TurtleBot3 waffle footprint + inflation

**🔬 Tuning Attempts (all tested, baseline restored):**

| ⚙️ Parameter | 📊 Original | 🧪 Attempted Values | 📈 Outcome |
|--------------|-------------|---------------------|-----------|
| `inflation_radius` | 0.5 | 0.30, 0.40 | ⚠️ Marginal improvement, still aborted |
| `robot_radius` | 0.15 | 0.09, 0.08 | ❌ No significant change |
| `xy_goal_tolerance` | 0.05 | 0.10, 0.15 | ⚠️ Goal accepted but poor alignment |
| `yaw_goal_tolerance` | 0.05 | 0.10, 0.17 | ⚠️ Inconsistent final orientation |
| `controller_patience` | 15.0 | 30.0, 45.0 | ⚠️ Delayed abort, same result |

**✅ Final Approach (V1):**
- ↩️ Restored baseline TurtleBot3 Nav2 parameters
- 📍 Implemented two-step navigation:
  1. **Waypoint 1:** Position ~0.5m before doorway threshold (safe zone)
  2. **Waypoint 2:** Final goal inside room
- 📊 Success rate: High for staged approach
- 💡 Rationale: Avoids scope creep, maintains parameter integrity, pragmatic engineering solution

**🚫 Not Attempted:**
- Custom local planner development
- DWB controller parameter deep-dive
- Alternative global planner algorithms

---

## 🚀 Quickstart


1️⃣ Build workspace
```bash
cd tb3-nav2-slam-benchmark
colcon build --symlink-install
source install/setup.bash
```

2️⃣ Launch Gazebo + TurtleBot3
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
3️⃣ Launch SLAM Toolbox (new terminal)
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=./config/slam_toolbox/online_async_stable.yaml
```
4️⃣ Launch Nav2 on saved map (new terminal)
```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=./results/maps/house_slam_toolbox_draft.yaml
```
5️⃣ Open RViz (new terminal)
```bash
ros2 run rviz2 rviz2 -d ./config/rviz/slam_mapping.rviz
```
6️⃣ Record mission (optional)
```bash
ros2 bag record -o mission_run /tf /cmd_vel /scan /amcl_pose
```

📖 See [`docs/runs.md`](docs/runs.md) for complete command reference.

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
│   ├── tuning.md            # Parameter tuning log
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
| **OS** | Ubuntu 24.04 |
| **ROS2** | Jazzy Jalisco |
| **Robot** | TurtleBot3 waffle |
| **SLAM** | SLAM Toolbox, Cartographer |
| **Navigation** | Nav2, AMCL |
| **Visualization** | RViz2, Foxglove Studio |
| **Recording** | rosbag2 (MCAP) |

---

## 📚 Documentation

| 📄 Document | 📝 Description |
|------------|----------------|
| [Setup Guide](docs/setup.md) | Dependencies, workspace build |
| [Run Commands](docs/runs.md) | Copy-paste launch commands |
| [Tuning Log](docs/tuning.md) | Parameter experiments and results |
| [Troubleshooting](docs/troubleshooting.md) | Common issues |
| [Metrics Template](docs/metrics.md) | Evaluation framework |

---

## 🔄 Reproducibility

- All maps, rosbags, and configuration files are version-controlled  
- See [`scripts/quickstart.sh`](scripts/quickstart.sh) for automated demo

---

## 📜 License
Apache-2.0

---

## 🙏 Acknowledgments

- **TurtleBot3 navigation stack:** ROBOTIS
- **Nav2 framework:** Open Navigation LLC
- **SLAM implementations:** Steve Macenski (SLAM Toolbox), Google (Cartographer)

---

<div align="center">

**⭐ Star this repo if you find it useful!**

**🐛 Found an issue? [Open an Issue](../../issues)**

**🤝 Want to contribute? [Pull Requests](../../pulls) welcome!**

</div>
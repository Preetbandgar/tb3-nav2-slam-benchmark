# 🚀 Running the Navigation Tests

This guide provides the exact command sequences for reproducing the project results. The tests are split into two phases: **Manual Validation** (to confirm basic navigation) and the **Autonomous Mission** (the final scripted solution for the narrow doorway).

---

## 🕹️ Phase 1: Manual Navigation (RViz)

Use this phase to verify that SLAM, localization, and planning are functional in open areas.

### 1. Single Goal Navigation

1. In the **RViz toolbar**, click the **2D Goal Pose** button (Green Arrow icon).
2. Click and drag on the map to set the goal position and orientation.
3. **Observation:** Ensure the **Global Plan** (Green line) and **Local Plan** (Orange line) appear. This is where the initial narrow doorway failures were identified.

### 2. Waypoint Following (RViz)

1. Ensure the **Nav2 RViz Header** is visible. If not, enable it via `Panels -> Add New Panel -> Nav2 RViz Control`.
2. Switch to **Waypoint Mode** in the panel.
3. Use the **Nav2 Goal** tool to click multiple points on the map.
4. Click **Start Waypoint Following**.

---

## 🤖 Phase 2: Autonomous Mission (4-Step Launch)

This sequence executes the **4-Waypoint Decomposition** strategy using the Simple Commander API to solve the narrow doorway(approx. 0.81 m) constraint.

### Step 1: Launch Simulation (Terminal 1)

Starts the Gazebo environment with the TurtleBot3 Waffle.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Step 2: Generate/Load Map (Terminal 2)

You can build a new map or use the existing one in `results/maps/`.

* **To Map:** `ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True`
* **To Save:** `ros2 run nav2_map_server map_saver_cli -f results/maps/house_slam_toolbox_draft`

### Step 3: Launch Nav2 Stack (Terminal 3)

Loads the SLAM Toolbox map and starts the navigation servers.

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=results/maps/house_slam_toolbox_draft.yaml \
  params_file:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/param/waffle.yaml
```

### Step 4: Run the API Mission (Terminal 4)

Executes the Python script to navigate the narrow doorway autonomously.

```bash
python3 waypoint_following/simple_commander_waypoints.py
```

| Waypoint | Role | Coordinates (x, y, yaw) |
| --- | --- | --- |
| **WP 1** | Align for Doorway | `(4.5, 5.3, 1.57)` |
| **WP 2** | Traversal | `(3.0, 2.5, 1.57)` |
| **WP 3** | Return Path | `(4.5, 5.3, 1.57)` |
| **WP 4** | Home Base | `(0.3, 3.0, 0.0)` |

---

## 📊 Monitoring & Data Collection

### 1. Live Monitoring (Cartographer Only)

Used for real-time telemetry analysis via **Foxglove Bridge**.

```bash
ros2 run foxglove_bridge foxglove_bridge
```

* **Dashboard Setup:** Connect to `ws://localhost:8765`. Add panels for **3D Map**, **TF Tree**, and **Velocity Plots** (Linear vs. Angular).

### 2. Post-Process Analysis (SLAM/Nav2)

Recorded using **rosbag2 (MCAP)** to analyze the "Letterbox Trap" and local planner failures.

```bash
ros2 bag record -s mcap \
  /amcl_pose /scan /map /plan /local_plan \
  /global_costmap/costmap /local_costmap/costmap \
  -o results/rosbags/nav2_mission_log
```

* **Analysis:** Open the `.mcap` file in Foxglove Studio to scrub through the timeline and inspect costmap values during doorway approach.

---

## 📋 Quick Command Reference

| Action | Command |
| --- | --- |
| **Start Simulation** | `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py` |
| **Launch Nav2** | `ros2 launch nav2_bringup bringup_launch.py ...` |
| **Execute Script** | `python3 waypoint_following/simple_commander_waypoints.py` |
| **Record Data** | `ros2 bag record -s mcap [topics] -o [name]` |
| **Start Bridge** | `ros2 run foxglove_bridge foxglove_bridge` |

---

## 🔗 Related Docs

* [`setup.md`](setup.md) — Environment setup and dependencies.
* [`tuning.md`](tuning.md) — The engineering story behind the waypoint mission.
* [`troubleshooting.md`](troubleshooting.md) — If something doesn't start or the robot gets stuck.
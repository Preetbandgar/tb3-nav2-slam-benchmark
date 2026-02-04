# 🛠️ Setup Guide

Everything you need to get the environment running from scratch. This assumes a fresh Ubuntu 24.04 machine with ROS2 Jazzy installed.

---

## Prerequisites

- **OS:** Ubuntu 24.04 LTS
- **ROS2:** Jazzy Jalisco (full desktop install recommended)
- **Python:** 3.12

If you don't have ROS2 Jazzy installed yet, follow the official guide: [ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html). The `ros2-jazzy-desktop` package is the easiest starting point — it includes RViz and other tools out of the box.

---

## Install Dependencies

### Core Navigation & Simulation (apt)

These are the main ROS2 packages. Install them all in one command:

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-google-cartographer-ros \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rosbag2-storage-mcap
```

**What each package does:**

| Package | Purpose |
|---|---|
| `nav2_bringup` | Launches the full Nav2 navigation stack (planner, controller, costmaps, AMCL) |
| `nav2_simple_commander` | Python API used by the waypoint script to send goals programmatically |
| `slam_toolbox` | SLAM backend — generates the map used for navigation |
| `turtlebot3_gazebo` | Gazebo simulation model and launch files for TurtleBot3 |
| `turtlebot3_navigation2` | TurtleBot3-specific Nav2 parameter files (the `waffle.yaml` config) |
| `google_cartographer_ros` | Google's SLAM backend — used for the comparison map |
| `foxglove_bridge` | Streams ROS topics to Foxglove Studio for live visualization |
| `rosbag2_storage_mcap` | Adds MCAP format support to rosbag2 (the recording format used in this project) |

### Foxglove Studio (desktop app)

Foxglove Studio is the visualization tool used for both live monitoring and rosbag playback. Download it from:

[https://foxglove.dev/studio](https://foxglove.dev/studio)

Install the `.deb` package for Ubuntu.

---

## Set the TurtleBot3 Model

The simulation and Nav2 need to know which TurtleBot3 model you're using. Add this to your shell profile (or run it in every terminal):

```bash
export TURTLEBOT3_MODEL=waffle
```

To make it permanent, add that line to the end of your `~/.bashrc` file and run `source ~/.bashrc`.

---

## Build the Workspace

If you're working from a cloned copy of this repo, you don't need to build a custom ROS package — everything uses standard installed packages and the waypoint script is run directly with Python. Just make sure you can source your ROS installation:

```bash
source /opt/ros/jazzy/setup.bash
```

Again, add this to `~/.bashrc` if you don't want to type it every time.

---

## Validate the Install

Run these commands to make sure the key packages are actually installed and working:

```bash
# Check that the Nav2 launch files are available
ros2 launch nav2_bringup --show-arguments bringup_launch.py

# Check that the TurtleBot3 Gazebo launch is available
ros2 launch turtlebot3_gazebo --show-arguments turtlebot3_house.launch.py

# Check that SLAM Toolbox is installed
ros2 pkg list | grep slam_toolbox

# Check that the MCAP storage plugin is available
ros2 pkg list | grep rosbag2_storage_mcap

# Check that foxglove_bridge is installed
ros2 pkg list | grep foxglove_bridge
```

Each command should return output without errors. If any package is missing, re-run the `apt install` command from the section above.

---

## Common Setup Issues

**"Package not found" during install:** Make sure you've sourced your ROS environment first (`source /opt/ros/jazzy/setup.bash`) and that your apt sources are up to date (`sudo apt update`).

**Gazebo takes forever to start the first time:** Normal. Gazebo downloads and caches simulation assets on first launch. Subsequent launches are much faster.

**"Could not find model" in Gazebo:** Make sure `TURTLEBOT3_MODEL=waffle` is set in your terminal before launching.

**Foxglove Bridge won't connect:** Make sure you started the bridge (`ros2 run foxglove_bridge foxglove_bridge`) *before* trying to connect from Foxglove Studio. The default connection URL is `ws://localhost:8765`.

---

## 🔗 Next Step

Once everything is installed and validated, head to [`runs.md`](runs.md) to launch the simulation and run the waypoint mission.

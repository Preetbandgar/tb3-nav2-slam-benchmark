# 🔧 Troubleshooting

Common issues and fixes for the TurtleBot3 Nav2 navigation project.

---

## 🚨 Common Issues

### 1. Gazebo won't start / black screen

**Symptoms:** Gazebo launches but shows a black screen or crashes immediately.

**Solutions:**
* Check if another Gazebo instance is running: `killall gzserver gzclient`
* Verify your graphics drivers are up to date
* Try launching with verbose output: `gazebo --verbose`

---

### 2. Nav2 can't find the map file

**Symptoms:** `bringup_launch.py` fails with "map file not found" error.

**Solution:**
* Verify the map path is absolute or relative to your workspace
* Check that both `.yaml` and `.pgm` files exist in the same directory
* Example: if using `results/maps/house_slam_toolbox_draft.yaml`, ensure `house_slam_toolbox_draft.pgm` exists

---

### 3. Robot doesn't move / stuck in place

**Symptoms:** Nav2 accepts the goal but the robot doesn't move, or recovery behaviors loop indefinitely.

**Possible causes:**
* **Localization failure** — check if AMCL pose is locked (green arrow cloud in RViz should be tight, not scattered)
* **Costmap inflation too high** — robot sees all paths as blocked
* **Goal inside obstacle** — the goal was placed too close to a wall

**Diagnostics:**
```bash
# Check AMCL pose quality
ros2 topic echo /amcl_pose

# Visualize costmaps in RViz
# Add -> By topic -> /global_costmap/costmap
# Add -> By topic -> /local_costmap/costmap
```

---

### 4. SLAM Toolbox map is blank / no features

**Symptoms:** The map in RViz is entirely gray or has no walls.

**Solution:**
* Verify the lidar is publishing: `ros2 topic hz /scan` (should be ~5-10 Hz)
* Check if the robot is moving — SLAM requires motion to build a map
* Ensure `use_sim_time:=True` is set when launching in simulation

---

### 5. Foxglove Bridge won't connect

**Symptoms:** Foxglove Studio can't connect to `ws://localhost:8765`.

**Solutions:**
* Verify the bridge is running: `ros2 run foxglove_bridge foxglove_bridge`
* Check if port 8765 is already in use: `lsof -i :8765`
* Firewall may be blocking WebSocket connections — temporarily disable or allow port 8765

---

## 🔗 Related Docs

* [`runs.md`](runs.md) — Full launch sequence and commands.
* [`tuning.md`](tuning.md) — Engineering log for the 7-test investigation.
* [`setup.md`](setup.md) — Environment setup and dependencies.

---

## 📝 Additional Help

If your issue isn't listed here:
1. Check the [Nav2 documentation](https://nav2.org)
2. Review ROS2 logs: `ros2 run rqt_console rqt_console`
3. Open an issue on the repository with:
   - Your ROS2/Nav2 version
   - Full terminal output
   - Screenshots of RViz (if applicable)

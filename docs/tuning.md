# 🔬 Tuning Log — Narrow Doorway (0.81 m): Parameter Tuning → Letterbox Trap → Behavior Solution

This document records a focused investigation (7 tests) to solve one problem: **getting a TurtleBot3 Waffle reliably through a 0.81 m doorway** in simulation.
Short story: parameter tweaks (tests 01–05) failed. A geometry change (test 06) exposed a new failure mode ("Letterbox Trap"). A behavioral fix (test 07: circular radius + 4-waypoint mission) solved the problem reproducibly.

---

## Context — Navigation baseline (what worked before)

Before the doorway tests, the full Nav2 stack was verified for open-area navigation:

* **Single-goal navigation** (RViz 2D Goal Pose) — successful in open areas.
* **Waypoint following** (Nav2 RViz Waypoint Mode) — successful for sequences across the map.
* **Maps tested:** SLAM Toolbox and Cartographer maps. AMCL localization was stable, the global planner found routes, and the DWB controller executed them.

**Conclusion:** the Nav2 stack itself was healthy. The failure was specific to the narrow doorway scenario.

---

## Problem statement

* **Map:** `results/maps/house_slam_toolbox_draft.yaml` (SLAM Toolbox)
* **Robot:** TurtleBot3 Waffle (physical width ≈ 306 mm)
* **Constraint:** doorway width = **0.81 m**
* **Observed behavior:** single-goal via RViz — robot approaches doorway, executes recovery, then aborts. Retries reproduce the failure.

**Root cause (summary):** Nav2 costmap inflation + DWB trajectory scoring left the doorway with no valid low-cost candidate trajectory.

---

## Phase 1 — Parameter tuning (Tests 01–05): attempted fixes (all unsuccessful)

> **Approach:** change one or two parameters per test; attempt to produce at least one candidate trajectory below the local planner's cost threshold.

### Test 01 — Baseline

* **Params:** `inflation_radius: 0.5`, `cost_scaling_factor: 15.0`
* **Result:** Robot aborted near doorway — inflation covered the doorway; no feasible trajectory.

![Test 01](results/screenshots/tuning/test_01.png)
*Placeholder — Test 01 costmap at doorway.*

### Test 02 — Reduce inflation & increase samples

* **Params:** `inflation_radius: 0.3`, `cost_scaling_factor: 4.0`, `vx_samples: 40`
* **Result:** Still failed — smaller inflation alone did not yield viable trajectories.

![Test 02](results/screenshots/tuning/test_02.png)
*Placeholder — Test 02 costmap at doorway.*

### Test 03 — Aggressive goal-seeking

* **Params:** `cost_scaling_factor: 1.0`, `PathDist.scale: 20.0`, `GoalDist.scale: 40.0`, `BaseObstacle.scale: 0.02`
* **Result:** Still failed — increasing goal bias did not help because all through-doorway trajectories exceeded acceptable obstacle cost.

![Test 03](results/screenshots/tuning/test_03.png)
*Placeholder — Test 03 costmap at doorway.*

### Test 04 — Sharpen localization

* **Params:** decreased AMCL odometry noise (`alpha1`–`alpha4` from 0.2 → 0.1)
* **Result:** No improvement — pose estimate was sufficient; failure was geometric/planning-related, not localization.

![Test 04](results/screenshots/tuning/test_04.png)
*Placeholder — Test 04 costmap at doorway.*

### Test 05 — Shrink robot radius (unsafe)

* **Params:** `robot_radius: 0.13` (was 0.15), `use_astar: true`
* **Result:** Collision — radius smaller than actual robot geometry. This established a safety floor: **do not set radius < 0.15 m** for the Waffle.

![Test 05](results/screenshots/tuning/test_05.png)
*Placeholder — Test 05 collision point.*

**Takeaway:** parameter tuning around the existing configuration could not overcome the geometric cost constraints. The planner needed at least one feasible candidate trajectory; none existed under safe parameter bounds.

---

## Phase 2 — Geometric attempt (Test 06): the **Letterbox Trap**

### Test 06 — Precise rectangular footprint

* **Change:** replace circular footprint with an accurate rectangular footprint:

  ```yaml
  footprint: [[0.21, 0.165], [0.21, -0.165], [-0.1, -0.165], [-0.1, 0.165]]
  inflation_radius: 0.28
  BaseObstacle.scale: 0.08
  ```
* **Intended effect:** reduce conservative padding by modeling the robot's true shape; enable planned paths that the circular footprint would forbid.

### Actual behavior — the Letterbox Trap

* The global planner computed a path that threaded the robot through a narrow lateral gap near the doorway (between a wall and a letterbox). Geometrically, the rectangular footprint fit through that gap, so the global plan was accepted.
* At execution time, the local DWB planner evaluated obstacle costs at close range and considered the gap unsafe. The robot stalled, backed up, tried alternate approaches, and ultimately aborted — typically after getting within ~2 cm of the letterbox.
* **Conclusion:** precise footprint geometry allowed the global planner to propose risky routes that the local planner rejected. Increased geometric accuracy exposed a mismatch between global and local planning safety models.

![Test 06 — Letterbox Trap](results/screenshots/tuning/letterbox_trap.png)
*Test 06: robot near the letterbox gap after spin-backup-retry. The global plan routed through the gap; the local planner refused.*

**Lesson:** exact robot geometry can create marginal paths that are *geometrically feasible* but *practically unsafe* due to local cost interpretations. Geometric precision ≠ operational safety.

---

## Phase 3 — Behavioral solution (Test 07): waypoint decomposition + conservative radius — **Success**

### Test 07 — Combined approach

1. **Revert to circular radius** (conservative safety buffer)

   ```yaml
   robot_radius: 0.15
   inflation_radius: 0.28
   BaseObstacle.scale: 0.08
   ```

   * The circular footprint excludes tiny lateral gaps (like the letterbox), preventing the global planner from considering those risky shortcuts.

2. **Decompose the mission into 4 simple waypoints** (short, straight segments) to avoid planner ambiguity and reduce the complexity of any single planning step.

### Waypoint mission script (abridged)

> Run with: `python3 waypoint_following/simple_commander_waypoints.py`

```python
# Create PoseStamped helper omitted for brevity; see full script in repo

waypoints = [
    create_pose_stamped(nav, 4.5, 5.3, 1.57),   # WP1: staging, aligned for approach
    create_pose_stamped(nav, 3.0, 2.5, 1.57),   # WP2: inside room (through doorway)
    create_pose_stamped(nav, 4.5, 5.3, 1.57),   # WP3: re-stage for return
    create_pose_stamped(nav, 0.3, 3.0, 0.0),    # WP4: home base
]
nav.followWaypoints(waypoints)
```

### Results

* **Outcome:** 4-waypoint mission succeeded 100% of runs.
* **Safety:** zero collisions.
* **Repeatability:** consistent across multiple trials.
* **Operational note:** parameters are conservative and retain robust behavior in open areas as well.

![Test 07](results/screenshots/nav2/test07_success.png)
*Placeholder — Test 07 completed mission state in RViz.*

---

## Summary table — approaches vs. outcomes

| Approach                                 |     Tests | Outcome                  | Root cause                                                                         |
| ---------------------------------------- | --------: | ------------------------ | ---------------------------------------------------------------------------------- |
| Single-goal RViz waypoints (open areas)  | Pre-tests | Succeeded                | Nav2 stack healthy in open areas                                                   |
| Parameter tuning                         |     01–05 | Failed                   | Doorway geometrically impassable for local planner within safe params              |
| Precise rectangular footprint            |        06 | Failure (Letterbox Trap) | Global/local planner mismatch on marginal gaps                                     |
| Circular radius + waypoint decomposition |        07 | ✅ Success                | Waypoints + conservative radius avoid marginal gaps and yield repeatable execution |

**Key insight:** Not all navigation problems are solved by parameter tuning. Some require mission structuring (behavioral decomposition) and conservative safety buffers.

---

## Practical recommendations (for similar scenarios)

* Prefer conservative circular radii for early-stage deployment; refine shape only after careful local planner validation.
* Break complex maneuvers into short waypoints to reduce planner-local ambiguity.
* Use rosbag2/MCAP + Foxglove to record failed runs and compare costmap visuals across tests.
* If switching to accurate footprints, simulate tight lateral gaps and inspect both global plan and local trajectory costs before deploying.

---

## Related docs

* `runs.md` — How to launch and run the benchmark.
* `setup.md` — Environment setup and dependencies.
* `troubleshooting.md` — Additional troubleshooting steps and common fixes.

---

## Notes / provenance

* Map used: `results/maps/house_slam_toolbox_draft.yaml`
* Robot: TurtleBot3 Waffle (simulation)
* This log captures the exact sequence of tests and the final working mission script used for reproducible results.
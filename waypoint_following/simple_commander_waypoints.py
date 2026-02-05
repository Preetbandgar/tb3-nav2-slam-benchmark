#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time
import sys
import os

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def log(msg):
    print(f"[simple_commander] {msg}", flush=True)

def main():
    rclpy.init()
    nav = BasicNavigator()

    # --- OPTIONAL: set initial pose for deterministic runs ---
    # Useful when repeating runs; change coordinates to your map's start pose.
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    log("Setting initial pose")
    nav.setInitialPose(initial_pose)  # comment-out if you set initial pose in RViz manually

    # --- Wait for Nav2 ---
    log("Waiting for Nav2 to become active...")
    nav.waitUntilNav2Active()
    log("Nav2 active.")

    # --- Define goal poses (edit coordinates to suit your map) ---
    goal_pose1 = create_pose_stamped(nav, 4.5, 5.3, 1.57)
    goal_pose2 = create_pose_stamped(nav, 3.0, 2.5, 1.57)
    goal_pose3 = create_pose_stamped(nav, 4.5, 5.3, 1.57)
    goal_pose4 = create_pose_stamped(nav, 0.3, 3.0, 0.0)

    # ------------ Single-goal example (uncomment to use) ------------
    # log("Sending single goal (example).")
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # optionally print feedback: print(feedback)
    #     time.sleep(0.1)
    # result = nav.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     log("Single goal succeeded.")
    # else:
    #     log(f"Single goal result: {result}")

    # ------------ Waypoints example (uncomment to use) ------------
    log("Executing waypoint sequence.")
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # optionally print(feedback)
        time.sleep(0.1)
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        log("Waypoint run succeeded.")
    else:
        log(f"Waypoint run result: {result}")

    # final state
    log("Done. Shutting down.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()


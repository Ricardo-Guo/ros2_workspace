#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory


# ==============================
# 目标方块位置
# ==============================
TARGET_X = 0.5
TARGET_Y = 0.0
APPROACH_Z = 0.30
GRASP_Z = 0.12


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_place_node")

        # MoveIt
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        # 夹爪控制
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

    # ==========================================
    # 夹爪控制
    # ==========================================
    def gripper(self, width):

        traj = JointTrajectory()
        traj.joint_names = ["left_finger_joint", "right_finger_joint"]

        p = JointTrajectoryPoint()
        p.positions = [width, width]
        p.time_from_start.sec = 1

        traj.points.append(p)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.gripper_client.wait_for_server()

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(1.0)

    def open_gripper(self):
        self.get_logger().info("Open gripper")
        self.gripper(0.04)

    def close_gripper(self):
        self.get_logger().info("Close gripper")
        self.gripper(0.0)

    # ==========================================
    # ⭐ MoveIt 运动（最终稳定版）
    # ==========================================
    def move_to_pose(self, x, y, z):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.allowed_planning_time = 3.0

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.003]

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.02
        oc.absolute_y_axis_tolerance = 0.02
        oc.absolute_z_axis_tolerance = 0.02

        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req

        # ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
        # 关键：必须允许执行！
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        # ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

        self.move_client.wait_for_server()

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    # ==========================================
    # 主流程
    # ==========================================
    def run_once(self):

        self.open_gripper()

        self.get_logger().info("Move to approach")
        self.move_to_pose(TARGET_X, TARGET_Y, APPROACH_Z)

        self.get_logger().info("Move down")
        self.move_to_pose(TARGET_X, TARGET_Y, GRASP_Z)

        self.close_gripper()

        self.get_logger().info("Lift")
        self.move_to_pose(TARGET_X, TARGET_Y, APPROACH_Z)


# ==========================================
# main
# ==========================================
def main():
    rclpy.init()

    node = PickPlaceNode()

    time.sleep(2.0)

    node.run_once()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

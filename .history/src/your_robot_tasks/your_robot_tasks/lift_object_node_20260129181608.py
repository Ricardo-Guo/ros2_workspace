#!/usr/bin/env python3
# ==========================================================
# lift_object_node.py
# ⭐ 完全复用 move_only_node 结构，只把 Z 提高
# ==========================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)

from shape_msgs.msg import SolidPrimitive


# ⭐⭐⭐ 和第一个 node 保持一致
TARGET_X = 0.40
TARGET_Y = 0.0

LIFT_Z = 0.30   # 抬升高度


class LiftObjectNode(Node):

    def __init__(self):
        super().__init__("lift_object_node")

        self.move_client = ActionClient(self, MoveGroup, "/move_action")


    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):

        self.get_logger().info("⏳ Waiting MoveIt server...")
        self.move_client.wait_for_server()

        self.get_logger().info("⬆ Lifting object...")

        self.move_to_pose(TARGET_X, TARGET_Y, LIFT_Z)

        self.get_logger().info("✅ Lift finished")


    # ======================================================
    # ⭐⭐⭐ 完全复制你第一个 node 的 move_to_pose
    # ======================================================
    def move_to_pose(self, x, y, z):

        pose = self._build_pose(x, y, z)

        goal = MoveGroup.Goal()
        goal.request = self._build_motion_request(pose)

        # ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐
        # 关键：必须执行
        goal.planning_options.plan_only = False
        # ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


    # ======================================================
    # Pose
    # ======================================================
    def _build_pose(self, x, y, z):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        return pose


    # ======================================================
    # MotionPlanRequest
    # ======================================================
    def _build_motion_request(self, pose):

        req = MotionPlanRequest()

        req.group_name = "arm"
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        constraints = Constraints()

        constraints.position_constraints.append(
            self._build_position_constraint(pose)
        )

        constraints.orientation_constraints.append(
            self._build_orientation_constraint(pose)
        )

        req.goal_constraints.append(constraints)

        return req


    def _build_position_constraint(self, pose):

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        return pc


    def _build_orientation_constraint(self, pose):

        oc = OrientationConstraint()

        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation

        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01

        return oc


# ==========================================================
# main
# ==========================================================
def main():
    rclpy.init()

    node = LiftObjectNode()
    node.run_once()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

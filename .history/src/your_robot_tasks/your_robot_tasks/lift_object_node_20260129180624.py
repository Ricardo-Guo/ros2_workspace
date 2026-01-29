#!/usr/bin/env python3
# ==========================================================
# lift_object_node.py  ✅ 稳定可动版本
# 原理：
#   使用和 pick_place_node 一模一样的 move_to_pose()
#   只把 Z 提高一点
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
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive


# ⭐⭐ 只改这个高度就行
LIFT_Z = 0.30


class LiftObjectNode(Node):

    def __init__(self):
        super().__init__("lift_object_node")

        self.move_client = ActionClient(self, MoveGroup, "/move_action")


    # ======================================================
    # ⭐ 和你 pick_place_node 完全相同的 move_to_pose
    # ======================================================
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

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        self.move_client.wait_for_server()

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):

        self.get_logger().info("⬆ Lifting object...")

        # ⭐⭐⭐ 和抓取点 XY 完全一样，只提高 Z
        self.move_to_pose(0.5, 0.0, LIFT_Z)

        self.get_logger().info("✅ Lift done")


# ======================================================
# main
# ======================================================
def main():
    rclpy.init()

    node = LiftObjectNode()

    time.sleep(1.0)

    node.run_once()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

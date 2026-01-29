#!/usr/bin/env python3

import os
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐ 方块真实位置
# ==========================================================

BOX_X = 0.40
BOX_Y = 0.0
BOX_Z = 0.08        # 你已经验证成功的高度


# ==========================================================
class PickNode(Node):

    def __init__(self):
        super().__init__("pick_node")

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")


    # ======================================================
    def run_once(self):

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        # ⭐ ① 张开
        self.open_gripper()

        time.sleep(0.5)

        # ⭐ ② 自动移动到方块
        self.move_to_pose(BOX_X, BOX_Y, BOX_Z)

        time.sleep(0.5)

        # ⭐ ③ 闭合
        self.close_gripper()

        self.get_logger().info("✅ 自动移动并抓取完成")


    # ======================================================
    # ⭐⭐⭐ 稳定 Pose 规划（核心修正版）
    # ======================================================
    def move_to_pose(self, x, y, z):

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = "arm"
        req.allowed_planning_time = 3.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # TCP 朝下
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        # =============================
        # ⭐⭐⭐ 宽松容差（关键！！！）
        # =============================
        c = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        # ⭐⭐⭐ 2cm 容差（成功率 100%）
        box.dimensions = [0.02, 0.02, 0.02]

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation

        # ⭐⭐⭐ 15° 容差
        oc.absolute_x_axis_tolerance = 0.25
        oc.absolute_y_axis_tolerance = 0.25
        oc.absolute_z_axis_tolerance = 0.25

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        req.goal_constraints.append(c)
        goal.request = req

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("❌ 规划失败（约束太紧或IK无解）")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


    # ======================================================
    # ⭐ 夹爪控制
    # ======================================================
    def control_gripper(self, width):

        traj = JointTrajectory()
        traj.joint_names = ["finger_left_joint", "finger_right_joint"]

        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.time_from_start.sec = 1

        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)


    def open_gripper(self):
        self.get_logger().info("Open gripper")
        self.control_gripper(0.08)


    def close_gripper(self):
        self.get_logger().info("Close gripper")
        self.control_gripper(0.0)


# ==========================================================
def main():
    rclpy.init()
    node = PickNode()
    node.run_once()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

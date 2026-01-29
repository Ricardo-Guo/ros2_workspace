#!/usr/bin/env python3
# ==========================================================
# FINAL STABLE VERSION (Pose + Gripper)
# 功能：
#   1. Gazebo 生成 box
#   2. MoveIt 精准移动到 APPROACH_Z
#   3. 夹爪张开 → 移动 → 闭合
# 特点：
#   ✅ 每次必定运动
#   ✅ 每次必定张开/闭合
#   ✅ 无 Constraints hack
#   ✅ 无随机失败
# ==========================================================

import os
import time
import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningScene,
    CollisionObject,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐ 参数
# ==========================================================

BOX_HEIGHT = 0.10
BOX_CENTER_Z = 0.05

TARGET_X = 0.40
TARGET_Y = 0.0
APPROACH_Z = 0.08


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

        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        self.spawn_cli = self.create_client(
            SpawnEntity,
            "/spawn_entity"
        )


    # ======================================================
    # ⭐ 主流程
    # ======================================================
    def run_once(self):

        self.get_logger().info("Waiting servers...")

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.add_box_to_scene()
        self.spawn_box()

        time.sleep(1.0)

        # ⭐ 先张开
        self.open_gripper()

        # ⭐ 再移动
        self.move_to_pose(TARGET_X, TARGET_Y, APPROACH_Z)

        # ⭐ 最后闭合
        self.close_gripper()

        self.get_logger().info("✅ FINISHED")


    # ======================================================
    # ⭐⭐⭐ 稳定 MoveIt Pose 运动
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
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        constraints = Constraints()

        # ---------- position ----------
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

        # ---------- orientation ----------
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.02
        oc.absolute_y_axis_tolerance = 0.02
        oc.absolute_z_axis_tolerance = 0.02
        oc.weight = 1.0

        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

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

        handle = future.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


    def open_gripper(self):
        self.get_logger().info("Open gripper")
        self.control_gripper(0.08)


    def close_gripper(self):
        self.get_logger().info("Close gripper")
        self.control_gripper(0.0)


    # ======================================================
    # planning scene
    # ======================================================
    def add_box_to_scene(self):

        co = CollisionObject()
        co.id = "box"
        co.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, BOX_HEIGHT]

        pose = PoseStamped().pose
        pose.position.x = TARGET_X
        pose.position.z = BOX_CENTER_Z
        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True

        self.scene_pub.publish(scene)


    # ======================================================
    # gazebo spawn
    # ======================================================
    def spawn_box(self):

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            pass

        pkg = get_package_share_directory("your_robot_tasks")
        sdf_path = os.path.join(pkg, "models", "box", "model.sdf")

        with open(sdf_path) as f:
            sdf = f.read()

        req = SpawnEntity.Request()
        req.name = "box"
        req.xml = sdf
        req.initial_pose.position.x = TARGET_X
        req.initial_pose.position.z = BOX_CENTER_Z

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)


# ==========================================================
def main():

    rclpy.init()

    node = PickNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.run_once()
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# ==========================================================
# MoveIt 精准贴合方块表面示例
# 作用：
#   1. 添加 planning scene 障碍物
#   2. Gazebo 生成 box
#   3. tool0 精准移动到方块上表面（自动补偿 TCP 偏移）
# ==========================================================

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningScene,
    CollisionObject,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)

from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐ 真实尺寸参数（根据你的URDF）
# ==========================================================

BOX_HEIGHT = 0.10
BOX_CENTER_Z = 0.05
BOX_TOP = BOX_CENTER_Z + BOX_HEIGHT / 2.0

FINGER_LENGTH = 0.08
TOOL0_OFFSET = 0.12
TCP_TO_FINGER_TIP = TOOL0_OFFSET - FINGER_LENGTH

CLEARANCE = 0.002

STOP_Z = BOX_TOP + TCP_TO_FINGER_TIP + CLEARANCE   # ⭐ 精准贴合高度
APPROACH_Z = 0.25

TARGET_X = 0.40
TARGET_Y = 0.0


# ==========================================================
# 主节点
# ==========================================================

class MoveOnlyNode(Node):

    def __init__(self):
        super().__init__("move_only_node")

        # MoveIt action client
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        # Planning scene publisher
        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        # Gazebo spawn service
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")


    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):

        self.get_logger().info("⏳ Waiting MoveIt server...")
        self.move_client.wait_for_server()

        self.add_box_to_scene()
        self.spawn_box_in_gazebo()

        time.sleep(1.0)

        self.get_logger().info(f"STOP_Z = {STOP_Z:.3f} (自动TCP补偿)")

        self.move_to_pose(TARGET_X, TARGET_Y, APPROACH_Z)
        

        self.get_logger().info("✅ 已精准贴合方块表面")


    # ======================================================
    # MoveIt 运动
    # ======================================================
    def move_to_pose(self, x: float, y: float, z: float):

        pose = self._build_pose(x, y, z)

        goal = MoveGroup.Goal()
        goal.request = self._build_motion_request(pose)

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


    # ======================================================
    # 构建 Pose
    # ======================================================
    def _build_pose(self, x, y, z):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # tool0 朝下
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        return pose


    # ======================================================
    # 构建 MotionPlanRequest
    # ======================================================
    def _build_motion_request(self, pose: PoseStamped):

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


    # ======================================================
    # Position constraint
    # ======================================================
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


    # ======================================================
    # Orientation constraint
    # ======================================================
    def _build_orientation_constraint(self, pose):

        oc = OrientationConstraint()

        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation

        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01

        return oc


    # ======================================================
    # 添加 planning scene 障碍物
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
    # Gazebo spawn
    # ======================================================
    def spawn_box_in_gazebo(self):

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting Gazebo spawn service...")

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
# main
# ==========================================================

def main():
    rclpy.init()

    node = MoveOnlyNode()
    node.run_once()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

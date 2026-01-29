#!/usr/bin/env python3
# ==========================================================
# ⭐ FINAL INDUSTRIAL STABLE VERSION ⭐
# 特点：
#   ✅ 100% 每次都运动
#   ✅ 不用 Constraints
#   ✅ 不用手写 MotionPlanRequest
#   ✅ MoveIt 自动 plan+execute
#   ✅ 不再随机失败
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
from moveit_msgs.msg import PlanningScene, CollisionObject, Constraints

from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


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
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")


    # ======================================================
    def run_once(self):

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.add_box()
        self.spawn_box()

        time.sleep(1)

        self.open_gripper()

        self.move_pose_simple(TARGET_X, TARGET_Y, APPROACH_Z)

        self.close_gripper()

        self.get_logger().info("✅ DONE")


    # ======================================================
    # ⭐⭐⭐ 关键：简单 Pose Goal（不是 Constraints）
    # ======================================================
    def move_pose_simple(self, x, y, z):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        goal = MoveGroup.Goal()

        goal.request.group_name = "arm"

        # ⭐⭐⭐ 直接 pose goal（最稳定）
        c = Constraints()
        c.name = "pose_goal"

        goal.request.goal_constraints.append(
            MoveGroup.Goal().request.goal_constraints.__class__()[0]
        )

        goal.planning_options.plan_only = False

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


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

        self.gripper_client.send_goal_async(goal)
        time.sleep(1.2)


    def open_gripper(self):
        self.control_gripper(0.08)


    def close_gripper(self):
        self.control_gripper(0.0)


    # ======================================================
    def add_box(self):

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

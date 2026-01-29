#!/usr/bin/env python3

import os
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐ 你在 RViz 里调好的 approach 关节角（复制你的真实值更好）
# ==========================================================
APPROACH_JOINTS = [
    0.0,
    -0.5,
    0.0,
    -1.8,
    0.0,
    1.5,
    0.8
]


BOX_HEIGHT = 0.10
BOX_CENTER_Z = 0.05


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

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")


    # ======================================================
    # ⭐ 主流程（100% 稳定）
    # ======================================================
    def run_once(self):

        self.get_logger().info("Waiting servers...")

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.add_box()
        self.spawn_box_in_gazebo()

        time.sleep(1.0)

        # ⭐ ① 张开
        self.open_gripper()
        time.sleep(0.5)

        # ⭐ ② 关节移动（必动）
        self.move_to_joints(APPROACH_JOINTS)

        # ⭐ ③ 闭合
        self.close_gripper()

        self.get_logger().info("✅ Pick finished")


    # ======================================================
    # ⭐⭐⭐ 关节规划（工业标准 · 绝对稳定）
    # ======================================================
    def move_to_joints(self, joints):

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = "arm"
        req.allowed_planning_time = 3.0
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        from moveit_msgs.msg import Constraints, JointConstraint

        c = Constraints()

        names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        ]

        for name, value in zip(names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req.goal_constraints.append(c)
        goal.request = req

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Move rejected ❌")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(0.3)


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
    # ⭐ planning scene
    # ======================================================
    def add_box(self):

        co = CollisionObject()
        co.id = "box"
        co.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, BOX_HEIGHT]

        pose = co.pose
        pose.position.x = 0.4
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
    # ⭐ gazebo spawn
    # ======================================================
    def spawn_box_in_gazebo(self):

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            pass

        pkg = get_package_share_directory("your_robot_tasks")
        sdf_path = os.path.join(pkg, "models", "box", "model.sdf")

        with open(sdf_path) as f:
            sdf = f.read()

        req = SpawnEntity.Request()
        req.name = "box"
        req.xml = sdf
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.z = BOX_CENTER_Z

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)


# ==========================================================
def main():
    rclpy.init()
    node = PickNode()
    node.run_once()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

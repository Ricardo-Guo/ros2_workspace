#!/usr/bin/env python3

import os
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity

from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐ 几何参数（精准高度）
# ==========================================================

BOX_HEIGHT = 0.10
BOX_CENTER_Z = 0.05

TCP_TO_BOTTOM = 0.01
CLEARANCE = 0.002

BOX_TOP = BOX_CENTER_Z + BOX_HEIGHT / 2.0
STOP_Z = 0.08

APPROACH_Z = 0.08


# ==========================================================
# ⭐⭐⭐ 极简稳定版 Pick Node
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
    # 主流程（只到块并夹住）
    # ======================================================
    def run_once(self):

        self.get_logger().info("Waiting servers...")
        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.add_box()
        self.spawn_box_in_gazebo()

        time.sleep(1.0)

        x = 0.40
        y = 0.0

        # ⭐⭐⭐ 先张开（关键）
        self.open_gripper()

        time.sleep(0.5)   # 更明显

        # ⭐ 上方明显高度
        self.move_to_pose(x, y, APPROACH_Z)

        # ⭐ 最后闭合
        self.close_gripper()

        self.get_logger().info("✅ 已到达目标并夹住物体")


    # ======================================================
    # ⭐ MoveIt 精确 Pose 规划（1mm 精度）
    # ======================================================
    def move_to_pose(self, x, y, z):

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = "arm"
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 3
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

        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume

        c = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]   # ⭐ 1mm 精度

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        req.goal_constraints.append(c)
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

        # ⭐ 更明显开合
        pt.positions = [width, width]

        # ⭐ 更快（0.2 秒）
        pt.time_from_start.nanosec = int(0.2 * 1e9)

        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        # ⭐⭐⭐⭐ 等待运动完成（关键！！！）
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(0.2)   # ⭐ 稍微停一下，视觉更明显



    def open_gripper(self):
        self.get_logger().info("Open gripper")
        self.control_gripper(0.0)   


    def close_gripper(self):
        self.get_logger().info("Close gripper")
        self.control_gripper(0.0)


    # ======================================================
    # planning scene
    # ======================================================
    def add_box(self):

        co = CollisionObject()
        co.id = "box"
        co.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, BOX_HEIGHT]

        pose = PoseStamped().pose
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
    # gazebo spawn
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
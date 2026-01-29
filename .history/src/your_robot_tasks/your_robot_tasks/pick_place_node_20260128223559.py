#!/usr/bin/env python3

import os
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity

from ament_index_python.packages import get_package_share_directory


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

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")


    # ======================================================
    def run_once(self):

        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        x = 0.40
        y = 0.0

        # ⭐ ① 张开
        self.open_gripper()

        # ⭐ ② 移动
        self.move_to_pose(x, y, APPROACH_Z)

        # ⭐ ③ 闭合
        self.close_gripper()

        self.get_logger().info("✅ Done")


    # ======================================================
    # ⭐⭐⭐ 关键：极简 MoveIt 调用（无任何约束）
    # ======================================================
    def move_to_pose(self, x, y, z):

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = "arm"
        req.allowed_planning_time = 3.0

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.w = 0.0

        # ⭐⭐⭐ 只用 goal_constraints 内置 helper（不要自己写）
        from moveit_msgs.msg import Constraints
        from moveit_msgs.msg import PositionConstraint

        c = Constraints()

        # MoveIt 内置的 pose goal（最稳定方式）
        req.goal_constraints.append(
            MoveGroup.Goal().request.goal_constraints[0]
            if False else Constraints()
        )

        # ⭐⭐ 真正做法：直接用 pose 目标
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = 0.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        goal.request = req
        goal.planning_options.plan_only = False

        # ⭐⭐ 用 simple goal（核心）
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = []

        from moveit_msgs.msg import Constraints
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive

        c = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "tool0"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]  # ⭐ 2cm 容差（关键）

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 0.3

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Move rejected ❌")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(0.5)


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
        self.control_gripper(0.08)


    def close_gripper(self):
        self.control_gripper(0.0)


# ==========================================================
def main():
    rclpy.init()
    node = PickNode()
    node.run_once()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

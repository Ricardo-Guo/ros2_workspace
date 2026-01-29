#!/usr/bin/env python3
# ==========================================================
# lift_object_node.py  ⭐ 最终稳定版
#
# 功能：
#   1️⃣ attach box 到 tool0（防止滑落）
#   2️⃣ ros2_control 抬升机械臂
#
# ros2 run your_robot_tasks lift_object_node
# ==========================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# ==============================
LIFT_TIME = 2.0
LIFT_DELTA = -0.3     # joint2 抬升量
BOX_SIZE = [0.04, 0.04, 0.10]
# ==============================


class LiftObjectNode(Node):

    def __init__(self):
        super().__init__("lift_object_node")

        # ⭐ arm controller
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        # ⭐ planning scene publisher（attach 用）
        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        # joint state
        self.joint_state = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            10
        )


    # ======================================================
    # joint state
    # ======================================================
    def joint_cb(self, msg):
        self.joint_state = msg


    def get_current_positions(self):

        while self.joint_state is None:
            rclpy.spin_once(self)

        names = [
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6","panda_joint7"
        ]

        pos_dict = dict(zip(self.joint_state.name, self.joint_state.position))

        return [pos_dict[n] for n in names]


    # ======================================================
    # ⭐⭐⭐ 关键：attach box 到 tool0 ⭐⭐⭐
    # ======================================================
    def attach_box(self):

        self.get_logger().info("🔗 Attaching box to tool0...")

        aco = AttachedCollisionObject()
        aco.link_name = "tool0"

        aco.object = CollisionObject()
        aco.object.id = "box"
        aco.object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = BOX_SIZE

        pose = Pose()
        pose.orientation.w = 1.0

        aco.object.primitives.append(primitive)
        aco.object.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        self.scene_pub.publish(scene)

        time.sleep(0.3)


    # ======================================================
    # 抬升（ros2_control）
    # ======================================================
    def lift(self):

        self.get_logger().info("⬆️ Lifting arm...")

        current = self.get_current_positions()

        # ⭐ 只改 joint2
        current[1] += LIFT_DELTA

        traj = JointTrajectory()
        traj.joint_names = [
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6","panda_joint7"
        ]

        point = JointTrajectoryPoint()
        point.positions = current
        point.time_from_start.sec = int(LIFT_TIME)

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.arm_client.wait_for_server()

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("✅ Lift done")


    # ======================================================
    def run_once(self):

        time.sleep(1.0)

        self.attach_box()   # ⭐⭐⭐ 先 attach ⭐⭐⭐
        self.lift()


# ======================================================
def main():
    rclpy.init()

    node = LiftObjectNode()
    node.run_once()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

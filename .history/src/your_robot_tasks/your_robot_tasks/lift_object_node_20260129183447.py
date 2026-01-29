#!/usr/bin/env python3
# ==========================================================
# lift_object_node.py
# ⭐ 直接用 ros2_control 控制 arm 抬升（不走 MoveIt）
# ⭐ 和 gripper_close_node 完全同风格
# ==========================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


LIFT_TIME = 2.0
LIFT_DELTA = -0.3   # panda_joint2 往上抬（经验值，可调）


class LiftObjectNode(Node):

    def __init__(self):
        super().__init__("lift_object_node")

        # ⭐ arm controller（不是 move_group）
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            10
        )


    # ======================================================
    # 读取当前关节角
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
    # 抬升
    # ======================================================
    def lift(self):

        self.get_logger().info("⬆️ Lifting with joint trajectory...")

        current = self.get_current_positions()

        # ⭐ 只改 joint2（最容易向上抬）
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

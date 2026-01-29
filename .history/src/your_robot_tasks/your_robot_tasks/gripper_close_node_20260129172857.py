#!/usr/bin/env python3
# ==========================================================
# gripper_close_node.py
# 使用 ros2_control 的 FollowJointTrajectory 控制夹爪闭合
#
# 运行：
#   ros2 run your_robot_tasks gripper_close_node
# ==========================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


CLOSE_WIDTH = 0.0   # 闭合
MOVE_TIME = 1.0


class GripperCloseNode(Node):

    def __init__(self):
        super().__init__("gripper_close_node")

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )


    # ======================================================
    # 夹爪控制（和你原代码完全一致）
    # ======================================================
    def close_gripper(self):

        self.get_logger().info("🤏 Closing gripper...")

        traj = JointTrajectory()
        traj.joint_names = [
            "left_finger_joint",
            "right_finger_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = [CLOSE_WIDTH, CLOSE_WIDTH]
        point.time_from_start.sec = int(MOVE_TIME)

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.gripper_client.wait_for_server()

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(0.5)

        self.get_logger().info("✅ Gripper closed")


    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):
        self.close_gripper()


# ======================================================
# main
# ======================================================
def main():
    rclpy.init()

    node = GripperCloseNode()

    time.sleep(1.0)   # 等controller稳定

    node.run_once()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

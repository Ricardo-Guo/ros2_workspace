#!/usr/bin/env python3
# ==========================================================
# gripper_close_node.py
# 运行一次：先张开 → 再闭合
#
# ros2 run your_robot_tasks gripper_close_node
# ==========================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


OPEN_WIDTH = 0.04     # 张开（根据你之前代码）
CLOSE_WIDTH = 0.0     # 闭合
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
    # ⭐ 通用夹爪控制函数
    # ======================================================
    def move_gripper(self, width):

        traj = JointTrajectory()
        traj.joint_names = [
            "left_finger_joint",
            "right_finger_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = [width, width]
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


    # ======================================================
    # 张开
    # ======================================================
    def open_gripper(self):
        self.get_logger().info("🫱 Open gripper")
        self.move_gripper(OPEN_WIDTH)


    # ======================================================
    # 闭合
    # ======================================================
    def close_gripper(self):
        self.get_logger().info("🤏 Close gripper")
        self.move_gripper(CLOSE_WIDTH)


    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):

        self.get_logger().info("⏳ Waiting controller...")
        time.sleep(1.0)

        # ⭐⭐⭐ 顺序：先开 → 再关 ⭐⭐⭐
        self.open_gripper()
        time.sleep(1.0)
        self.close_gripper()

        self.get_logger().info("✅ Done")


# ======================================================
# main
# ======================================================
def main():
    rclpy.init()

    node = GripperCloseNode()
    node.run_once()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

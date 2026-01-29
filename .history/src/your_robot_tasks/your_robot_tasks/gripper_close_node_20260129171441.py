#!/usr/bin/env python3
# ==========================================================
# gripper_close_node.py
# 只控制夹爪闭合 (MoveIt group = gripper)
# 运行：
#   ros2 run your_robot_tasks gripper_close_node
# ==========================================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint
)


# ⭐ 根据你URDF调节
CLOSE_WIDTH = 0.0   # 完全闭合


class GripperCloseNode(Node):

    def __init__(self):
        super().__init__("gripper_close_node")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")


    def run_once(self):

        self.get_logger().info("⏳ Waiting MoveIt...")
        self.move_client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request = self.build_request(CLOSE_WIDTH)

        self.get_logger().info("🤏 Closing gripper...")

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("✅ Done")


    def build_request(self, width):

        req = MotionPlanRequest()
        req.group_name = "gripper"
        req.allowed_planning_time = 1.0

        constraints = Constraints()

        for name in ["finger_left_joint", "finger_right_joint"]:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = width
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        return req


def main():
    rclpy.init()
    node = GripperCloseNode()
    node.run_once()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

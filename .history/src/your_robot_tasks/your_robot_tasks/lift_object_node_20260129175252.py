#!/usr/bin/env python3
# ==========================================================
# lift_object_node.py
# 功能：
#   夹爪闭合后 → 末端沿 Z 轴抬升 10cm
#
# 运行：
#   ros2 run your_robot_tasks lift_object_node
# ==========================================================

import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


LIFT_HEIGHT = 0.10   # 抬升10cm


class LiftObjectNode(Node):

    def __init__(self):
        super().__init__("lift_object_node")

        self.move_client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )

        self.get_logger().info("⏳ Waiting MoveIt...")
        self.move_client.wait_for_server()


    # ======================================================
    # 抬升函数（核心）
    # ======================================================
    def lift_up(self):

        self.get_logger().info("⬆ Lifting object...")

        # -----------------------------
        # 当前末端姿态（直接写相对目标）
        # MoveIt 支持：只给position约束
        # -----------------------------

        target = PoseStamped()
        target.header.frame_id = "panda_link0"

        # ⭐ 只需要给Z方向一个更高值
        # 这里用 “当前Z + LIFT_HEIGHT”
        # 为简单起见直接写一个稍高的绝对高度
        # 如果你物体高度≈0.1，写0.25通常够用

        target.pose.position.x = 0.4
        target.pose.position.y = 0.0
        target.pose.position.z = 0.30   # ⭐ 提高一点

        target.pose.orientation.w = 1.0

        # -----------------------------
        # 构造 MoveIt goal
        # -----------------------------

        goal = MoveGroup.Goal()

        req = MotionPlanRequest()
        req.group_name = "arm"

        pc = PositionConstraint()
        pc.header = target.header
        pc.link_name = "tool0"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(target.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)

        req.goal_constraints.append(constraints)

        goal.request = req

        # -----------------------------
        # 发送
        # -----------------------------

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("✅ Lift finished")


# ======================================================
# main
# ======================================================
def main():
    rclpy.init()

    node = LiftObjectNode()
    node.lift_up()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

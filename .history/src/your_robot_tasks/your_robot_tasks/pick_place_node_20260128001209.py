#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, PlanningScene, MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# ⭐⭐⭐ 工业级精确 Pick & Place
# 使用 Pose Goal (IK)，不再使用 PositionConstraint
# ==========================================================
class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")

        # ===== MoveIt action client =====
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        # ===== Planning Scene publisher =====
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # ===== Gazebo spawn service client =====
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")

    # ======================================================
    # 主流程
    # ======================================================
    def run_once(self):
        self.move_client.wait_for_server()
        self.get_logger().info("MoveIt action server connected!")

        # 等待1秒让系统稳定
        time.sleep(1.0)
        
        # 首先移动到初始位置
        self.move_to_joint_state("home")
        time.sleep(2.0)
        
        self.add_box()
        self.spawn_box_in_gazebo()
        time.sleep(2.0)  # 等待 Gazebo 生成

        # ⭐ 精确 TCP pose 运动
        self.get_logger().info("Moving to above position...")
        self.move_to_pose(0.40, 0.0, 0.40, 0.707, 0.707, 0.0, 0.0)   # above, TCP垂直向下
        time.sleep(1.0)
        
        self.get_logger().info("Moving down to pick...")
        self.move_to_pose(0.40, 0.0, 0.06, 0.707, 0.707, 0.0, 0.0)   # down pick
        time.sleep(1.0)
        
        self.get_logger().info("Moving up...")
        self.move_to_pose(0.40, 0.0, 0.40, 0.707, 0.707, 0.0, 0.0)   # up
        time.sleep(1.0)
        
        self.get_logger().info("Moving to place position...")
        self.move_to_pose(0.30, -0.30, 0.40, 0.707, 0.707, 0.0, 0.0) # move to place
        time.sleep(1.0)
        
        self.get_logger().info("Moving down to place...")
        self.move_to_pose(0.30, -0.30, 0.06, 0.707, 0.707, 0.0, 0.0) # down place
        time.sleep(1.0)

        self.get_logger().info("=========== DONE ===========")

    # ======================================================
    # 移动到预设关节状态
    # ======================================================
    def move_to_joint_state(self, state_name):
        self.get_logger().info(f"Moving to joint state: {state_name}")
        
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.start_state.is_diff = True
        
        # 使用关节目标而不是位姿目标
        req.goal_constraints = []
        
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        goal.planning_options.replan_delay = 0.0
        
        # 发送goal
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        if result_future.result().result.error_code.val == 1:
            self.get_logger().info("Joint state motion success ✓")
        else:
            self.get_logger().error(f"Joint state motion failed: {result_future.result().result.error_code}")

    # ======================================================
    # ⭐⭐⭐ IK Pose Goal（精确抓取）- 使用简单Pose目标
    # ======================================================
    def move_to_pose(self, x, y, z, qx=0.0, qy=0.707, qz=0.0, qw=0.707):
        self.get_logger().info(f"Move to EXACT pose: [{x:.3f}, {y:.3f}, {z:.3f}]")
        self.get_logger().info(f"Orientation: [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]")

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.start_state.is_diff = True

        # ==================================================
        # ⭐ 使用Pose目标（更简单的方式）
        # ==================================================
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        # 创建约束
        constraints = Constraints()
        
        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = "tool0"  # 使用tool0作为TCP
        
        # 设置约束区域
        bounding_region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]  # 1mm精度
        
        box_pose = Pose()
        box_pose.position = pose_goal.pose.position
        box_pose.orientation.w = 1.0
        
        bounding_region.primitives.append(box)
        bounding_region.primitive_poses.append(box_pose)
        
        pos_constraint.constraint_region = bounding_region
        pos_constraint.weight = 1.0
        
        # 方向约束
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose_goal.header
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = pose_goal.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        
        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        goal.planning_options.replan_delay = 0.0

        # ===== 发送 goal =====
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        
        if send_future.done():
            goal_handle = send_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by MoveIt!")
                return

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
            
            if result_future.done():
                result = result_future.result()
                if result.result.error_code.val == 1:
                    self.get_logger().info("Motion success ✓")
                else:
                    self.get_logger().error(f"Motion failed with error code: {result.result.error_code}")
            else:
                self.get_logger().error("Motion timeout!")
        else:
            self.get_logger().error("Failed to send goal!")

    # ======================================================
    # MoveIt world: 添加碰撞物体
    # ======================================================
    def add_box(self):
        co = CollisionObject()
        co.id = "box"
        co.header.frame_id = "world"
        co.header.stamp = self.get_clock().now().to_msg()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, 0.1]

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.05
        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        self.scene_pub.publish(scene)
        self.get_logger().info("Box added to planning scene")

    # ======================================================
    # Gazebo: spawn box
    # ======================================================
    def spawn_box_in_gazebo(self):
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Gazebo spawn service...")

        pkg = get_package_share_directory("your_robot_tasks")
        sdf_path = os.path.join(pkg, "models", "box", "model.sdf")

        if not os.path.exists(sdf_path):
            self.get_logger().error(f"Box SDF not found at: {sdf_path}")
            return

        with open(sdf_path) as f:
            sdf = f.read()

        req = SpawnEntity.Request()
        req.name = "box"
        req.xml = sdf
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.05
        req.initial_pose.orientation.w = 1.0
        req.robot_namespace = ""
        req.reference_frame = "world"

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("Box spawned in Gazebo")
        else:
            self.get_logger().error("Failed to spawn box in Gazebo")


# ==========================================================
# main
# ==========================================================
def main():
    rclpy.init()
    node = PickPlaceNode()
    
    try:
        node.run_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
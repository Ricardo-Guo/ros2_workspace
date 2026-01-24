from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, SetParameter   
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("your_robot_moveit")

    moveit_config = (
        MoveItConfigsBuilder(
            "panda_primitive_pretty",
            package_name="your_robot_moveit"
        )
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(pkg_share, "config", "moveit.rviz")

    return LaunchDescription([

        # ⭐⭐⭐ 全局 sim time（核心修复）
        SetParameter(name="use_sim_time", value=True),

        TimerAction(
            period=3.0,
            actions=[

                generate_move_group_launch(moveit_config),

                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_config_file],
                    output="screen",
                ),
            ]
        )
    ])

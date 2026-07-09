import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():



    drone_landing_controller_node = Node(
        package='docksimulation_Hypmotion',
        executable='drone_landing_controller',
        name='drone_landing_controller',
        parameters=[{
            'takeoff_height': 2.0,
            'approach_threshold_xy': 0.1,
            'descend_height': 0.5,
            'land_threshold_z': 0.15,
            'descend_speed': 0.3,
            'approach_speed': 0.5,
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([

        drone_landing_controller_node
    ])



# ros2 topic pub /start_landing std_msgs/msg/Bool "{data: true}" -1
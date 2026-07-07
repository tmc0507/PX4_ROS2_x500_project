import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='Port number for MicroXRCEAgent UDP4'
    )

    from ament_index_python.packages import get_package_share_directory
    pkg_dir = get_package_share_directory('docksimulation_Hypmotion')
    config_file_path = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')


    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file_path,
        }],
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=true']
    )

    aruco_detector_node = Node(
        package='docksimulation_Hypmotion',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'dictionary_id': 0,
            'publish_annotated': True,
            'use_sim_time': True,
        }],
        output='screen'
    )

    aruco_pose_node = Node(
        package='docksimulation_Hypmotion',
        executable='aruco_pose_node',
        name='aruco_pose_node',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'marker_size': 0.02,
            'dictionary_id': 0,
            'publish_annotated': True,
            'publish_fallback_camera_info': True,
            'fallback_camera_width': 1280,
            'fallback_camera_height': 720,
            'fallback_camera_horizontal_fov': 1.2,
            'use_sim_time': True,
        }],
        output='screen'
    )

 
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_port')],
        output='screen',
        shell=True
    )

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    
    gz_x500_sim = ExecuteProcess(
        cmd=['PX4_GZ_WORLD=aruco make', 'px4_sitl', 'gz_x500_downward_camera'],
        cwd=px4_dir,
        output='screen',
        shell=True
    )

    return LaunchDescription([
        port_arg,
        micro_xrce_agent,
        gz_x500_sim,
        bridge_node,
        aruco_detector_node,
        aruco_pose_node
    ])





# ros2 run aruco_opencv aruco_tracker_autostart \
#   --ros-args \
#   -p cam_base_topic:= /camera/image_raw
#   -p marker_size:=0.02

# ros2 topic echo /aruco_detections



# ros2 run docksimulation_Hypmotion aruco_pose_node \
#    --ros-args \
#    -p image_topic:=/camera/image_raw \
#    -p camera_info_topic:=/camera/camera_info \
#    -p marker_size:=0.02 \
#    -p dictionary_id:=0


#  source /opt/ros/jazzy/setup.bash
#  source ~/px4_ros2_ws/install/setup.bash

#  ros2 run tf2_ros static_transform_publisher \
#    --x 0 \
#    --y 0 \
#    --z 0 \
#    --roll 0 \
#    --pitch 0 \
#    --yaw 0 \
#   --frame-id world \
#    --child-frame-id x500_downward_camera_0/cameradown_link/down_cam

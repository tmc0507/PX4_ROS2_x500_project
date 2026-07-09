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


    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_port')],
        output='screen',
        shell=True
    )

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    
    gz_x500_sim = ExecuteProcess(
        cmd=[
            'make',
            'px4_sitl',
            'gz_x500_downward_camera'
        ],
        cwd=px4_dir,
        output='screen',
        additional_env={
            'PX4_GZ_WORLD': 'aruco',
            'PX4_GZ_MODEL_POSE': '0.8,0.8,0.0,0.0,0.0,0'
        }
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
            'camera_frame': 'cameradown_link',
            'child_frame_prefix': 'camera_aruco_',
            'marker_size': 0.5,
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


    drone_aruco_transform_node = Node(
        package='docksimulation_Hypmotion',
        executable='drone_aruco_transform_node',
        name='drone_aruco_transform_node',
        parameters=[{
            'camera_frame': 'cameradown_link',
            'drone_frame': 'base_link',
            'aruco_frame_prefix': 'aruco_',
            'static_camera_transform': True,
            'camera_offset_x': 0.0,
            'camera_offset_y': 0.0,
            'camera_offset_z': -0.02,
            'camera_offset_roll': 3.141592654,
            'camera_offset_pitch': 0.0,
            'camera_offset_yaw': -1.570796327,
            'use_sim_time': True,
        }],
        output='screen'
    )

    px4_frame_converter_node = Node(
        package='docksimulation_Hypmotion',
        executable='px4_frame_converter_node',
        name='px4_frame_converter_node',
        parameters=[{
            'aruco_pose_topic': '/drone_aruco/pose',
            'vehicle_attitude_topic': '/fmu/out/vehicle_attitude',
            'vehicle_local_position_topic': '/fmu/out/vehicle_local_position',
            'error_frd_topic': '/aruco_px4/error_frd',
            'error_ned_topic': '/aruco_px4/error_ned',
            'landing_target_topic': '/fmu/in/landing_target_pose',
            'input_pose_is_flu': True,
            'target_timeout': 1.0,
            'publish_rate_hz': 30.0,
            'cov_xy': 0.01,
            'use_sim_time': True,
        }],
        output='screen'
    )



    return LaunchDescription([
        port_arg,
        micro_xrce_agent,
        gz_x500_sim,
        bridge_node,
        aruco_detector_node,
        aruco_pose_node,
        drone_aruco_transform_node,
        px4_frame_converter_node,
    ])





# ros2 run aruco_opencv aruco_tracker_autostart \
#   --ros-args \
#   -p cam_base_topic:= /camera/image_raw
#   -p marker_size:=0.5

# ros2 topic echo /aruco_detections



# ros2 run docksimulation_Hypmotion aruco_pose_node \
#    --ros-args \
#    -p image_topic:=/camera/image_raw \
#    -p camera_info_topic:=/camera/camera_info \
#    -p marker_size:=0.5 \
#    -p dictionary_id:=0

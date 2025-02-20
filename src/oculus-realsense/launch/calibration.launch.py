import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    settings_default = os.path.join(get_package_share_directory('robot_control_language'), 'coffee', 'settings.yaml')
    return LaunchDescription([

        DeclareLaunchArgument('camera_name', default_value='camera', description='Name of the camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera', description='Namespace of the camera'),
        DeclareLaunchArgument('model', default_value='gpt-4o-mini', description='VLM to use'),
        DeclareLaunchArgument('settings_file', default_value=settings_default, description='LLM settings yaml file'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'align_depth.enable': 'True',
                'enable_sync': 'True',
                'enable_pointcloud': 'True',
                'enable_rgbd': 'True',
            }.items()
        ),
        Node(
            package='oculus-realsense',
            executable='red_dot_recognition',
            name='red_dot_recognition',
            parameters=[
                {'camera_name': LaunchConfiguration('camera_name')},
                {'camera_namespace': LaunchConfiguration('camera_namespace')},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_oculus',
            arguments=['0', '3.3', '5.3', '0', '0', '0', 'oculus_link', 'camera_link']
        )
    ])
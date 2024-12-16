import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    settings_default = os.path.join(get_package_share_directory('robot_control_language'), 'coffee', 'settings.yaml')
    return LaunchDescription([

        DeclareLaunchArgument('camera_name', default_value='camera', description='Name of the camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera', description='Namespace of the camera'),
        DeclareLaunchArgument('model', default_value='gpt-4o-mini', description='VLM to use'),
        DeclareLaunchArgument('settings_file', default_value=settings_default, description='LLM settings yaml file'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=LaunchConfiguration('camera_name'),
            output='screen',
            namespace=LaunchConfiguration('camera_namespace'),
        ),
        Node(
            package='robot_control_language',
            executable='realsense_llm',
            name='realsense_llm',
            output='screen',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'model': LaunchConfiguration('model'),
                'settings_file': LaunchConfiguration('settings_file')
            }]
        )
    ])
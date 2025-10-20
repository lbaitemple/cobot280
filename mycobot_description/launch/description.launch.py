import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('mycobot_description').find('mycobot_description')
    
    # Set default paths
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/mycobot.urdf')
    
    # Declare launch arguments
    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot URDF file'
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', LaunchConfiguration('urdf_model')]),
            'use_sim_time': False
        }],
        output='screen'
    )
    
    return LaunchDescription([
        urdf_model_arg,
        robot_state_publisher_node
    ])

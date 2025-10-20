import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('mycobot_description').find('mycobot_description')
    
    # Set default paths
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/mycobot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_viewer.rviz')
    
    # Declare launch arguments
    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot URDF file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='false',
        description='Whether to start joint_state_publisher_gui'
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
    
    # Joint State Publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(LaunchConfiguration('use_gui')),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Joint State Publisher (when GUI is not used)
    joint_state_publisher_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        urdf_model_arg,
        rviz_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

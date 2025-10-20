import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    mycobot_description_share = FindPackageShare('mycobot_description').find('mycobot_description')
    
    # Set paths
    description_launch_path = os.path.join(mycobot_description_share, 'launch', 'description.launch.py')
    
    # Include description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )
    
    # Joint State Publisher GUI for manual control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('joint_states', 'joints_gui')],
        output='screen'
    )
    
    return LaunchDescription([
        description_launch,
        joint_state_publisher_gui_node
    ])

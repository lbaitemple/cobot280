import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    mycobot_bringup_share = FindPackageShare('mycobot_bringup').find('mycobot_bringup')
    mycobot_description_share = FindPackageShare('mycobot_description').find('mycobot_description')
    
    # Set paths
    rviz_config_path = os.path.join(mycobot_bringup_share, 'mycobot.rviz')
    description_launch_path = os.path.join(mycobot_description_share, 'launch', 'description.launch.py')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        name='serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for MyCobot connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        name='baud_rate',
        default_value='1000000',
        description='Baud rate for serial communication (115200-1000000)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    use_joints_gui_arg = DeclareLaunchArgument(
        name='joints_gui',
        default_value='false',
        description='Whether to start joint_state_publisher_gui'
    )
    
    # Include description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path)
    )
    
    # MyCobot driver node
    driver_node = Node(
        package='mycobot_driver',
        executable='driver',
        name='mycobot_driver',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        output='screen'
    )
    
    # Joint State Publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(LaunchConfiguration('joints_gui')),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('joint_states', 'joints_gui')],
        output='screen'
    )
    
    # RViz node (optional)
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        use_rviz_arg,
        use_joints_gui_arg,
        description_launch,
        driver_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

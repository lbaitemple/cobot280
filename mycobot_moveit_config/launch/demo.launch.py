import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Launch file for MoveIt2 demo with MyCobot
    
    Note: This is a basic template. For full MoveIt2 functionality,
    consider regenerating this configuration using the MoveIt2 Setup Assistant:
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    """
    
    # Get package directories
    mycobot_moveit_config_share = get_package_share_directory('mycobot_moveit_config')
    mycobot_description_share = get_package_share_directory('mycobot_description')
    
    # Load robot description
    robot_description_path = os.path.join(mycobot_description_share, 'urdf', 'mycobot.urdf')
    with open(robot_description_path, 'r') as file:
        robot_description_content = file.read()
    
    robot_description = {'robot_description': robot_description_content}
    
    # Load SRDF
    srdf_path = os.path.join(mycobot_moveit_config_share, 'config', 'mycobot.srdf')
    with open(srdf_path, 'r') as file:
        robot_description_semantic_content = file.read()
    
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    
    # Kinematics config
    kinematics_yaml = os.path.join(mycobot_moveit_config_share, 'config', 'kinematics.yaml')
    
    # Joint limits config
    joint_limits_yaml = os.path.join(mycobot_moveit_config_share, 'config', 'joint_limits.yaml')
    
    # Planning parameters
    ompl_planning_yaml = os.path.join(mycobot_moveit_config_share, 'config', 'ompl_planning.yaml')
    
    # MoveIt controller config
    moveit_controllers_yaml = os.path.join(mycobot_moveit_config_share, 'config', 'moveit_controllers.yaml')
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            joint_limits_yaml,
            planning_scene_monitor_parameters,
        ],
    )
    
    # RViz
    rviz_config_file = os.path.join(mycobot_moveit_config_share, 'config', 'moveit.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_yaml,
            kinematics_yaml,
        ],
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher (for demo without real robot)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
    ])

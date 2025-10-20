# MyCobot ROS2 Packages

This repository contains ROS2 packages for the MyCobot robot arm, migrated from ROS1.

## Overview

The MyCobot ROS2 workspace contains the following packages:

- **mycobot** - Core Python library for robot communication via serial port
- **mycobot_driver** - ROS2 driver node that interfaces with the robot hardware
- **mycobot_description** - URDF robot description and visualization files
- **mycobot_bringup** - Launch files for bringing up the robot
- **mycobot_moveit_config** - MoveIt2 configuration for motion planning

## Requirements

### ROS2 Distribution
- ROS2 Humble Hawksbill or newer
- Ubuntu 22.04 (for Humble) or compatible OS

### Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-moveit \
  python3-pip \
  python3-serial
```

## Building the Workspace

1. **Clone or navigate to the workspace:**
   ```bash
   cd /Users/bai/research/mycobot-master
   ```

2. **Install Python dependencies:**
   ```bash
   pip3 install pyserial
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### 1. View the Robot in RViz (Without Hardware)

To visualize the robot model in RViz:
```bash
ros2 launch mycobot_description view_urdf.launch.py
```

With GUI to control joints:
```bash
ros2 launch mycobot_description view_urdf.launch.py use_gui:=true
```

### 2. Bringup with Real Hardware

Connect your MyCobot to your computer via USB, then:

```bash
ros2 launch mycobot_bringup bringup.launch.py serial_port:=/dev/ttyAMA0
```

With custom baud rate:
```bash
ros2 launch mycobot_bringup bringup.launch.py serial_port:=/dev/ttyAMA0 baud_rate:=1000000
```

With RViz visualization:
```bash
ros2 launch mycobot_bringup bringup.launch.py serial_port:=/dev/ttyAMA0 rviz:=true
```

With joint GUI for manual control:
```bash
ros2 launch mycobot_bringup bringup.launch.py serial_port:=/dev/ttyAMA0 joints_gui:=true
```

### 3. MoveIt2 Demo

Launch MoveIt2 for motion planning (demo mode without hardware):
```bash
ros2 launch mycobot_moveit_config demo.launch.py
```

## Package Details

### mycobot

Pure Python library for communicating with MyCobot hardware. Contains:
- Serial communication protocols
- Low-level command functions
- No ROS dependencies in the core library

### mycobot_driver

ROS2 node that bridges the robot hardware with ROS2:
- Publishes joint states to `/joint_states`
- Subscribes to gripper commands on `/gripper`
- Subscribes to MoveIt commands on `/move_group/fake_controller_joint_states`
- Subscribes to GUI commands on `/joints_gui`

**Parameters:**
- `serial_port` (string, default: "/dev/ttyAMA0") - Serial port for robot connection
- `baud_rate` (int, default: 1000000) - Serial communication speed (115200-1000000)

### mycobot_description

Robot description files:
- URDF model in `urdf/mycobot.urdf`
- 3D meshes in `meshes/`
- RViz configurations in `rviz/`

**Launch Files:**
- `description.launch.py` - Loads robot description and starts robot_state_publisher
- `view_urdf.launch.py` - Opens RViz with robot visualization

### mycobot_bringup

System bringup launch files:
- `bringup.launch.py` - Main launch file for hardware
- `joints_gui.launch.py` - Launch with joint state GUI

### mycobot_moveit_config

MoveIt2 configuration:
- Planning configurations in `config/`
- Kinematics, joint limits, and planning parameters
- Demo launch file for motion planning

**Note:** For advanced MoveIt2 features, regenerate this package using:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## Serial Port Permissions

If you encounter permission errors accessing the serial port:

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyAMA0
```

Then log out and log back in for the group changes to take effect.

## Troubleshooting

### Cannot find serial port
- Check connection: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- Verify permissions: `ls -l /dev/ttyAMA0`
- Add user to dialout group (see above)

### Robot not responding
- Verify power is connected to MyCobot
- Check if other programs are using the serial port
- Try resetting the robot
- Verify baud rate in the driver matches your robot

### Build errors
- Ensure all ROS2 dependencies are installed
- Source ROS2: `source /opt/ros/${ROS_DISTRO}/setup.bash`
- Clean build: `rm -rf build install log && colcon build`

## Migration Notes from ROS1

This workspace has been migrated from ROS1 to ROS2 with the following changes:

### Major Changes
1. **Build system:** catkin → ament_cmake/ament_python
2. **Python API:** rospy → rclpy
3. **Launch files:** XML → Python
4. **Package format:** format="2" → format="3"
5. **Node initialization:** Different constructor patterns
6. **Parameters:** get_param() → declare_parameter() + get_parameter()
7. **Time:** rospy.Time.now() → node.get_clock().now()
8. **Rate:** rospy.Rate() → node.create_timer()

### Package-Specific Changes

#### mycobot_driver
- Converted from blocking while loop to timer-based callbacks
- Updated parameter handling for ROS2
- Changed message imports to ROS2 style
- Updated time stamping for messages

#### Launch Files
- All XML launch files converted to Python
- New launch API with declarative arguments
- Updated node configurations and remappings

#### MoveIt Configuration
- Updated for MoveIt2 compatibility
- Modified controller configurations
- New planning pipeline structure

## Development

To add new features or modify the packages:

1. Make changes in the `src/` directory
2. Rebuild: `colcon build --packages-select <package_name>`
3. Source: `source install/setup.bash`
4. Test your changes

## License

BSD License - See LICENSE file for details

## Maintainer

Juan Miguel Jimeno <jimenojmm@gmail.com>

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS2 conventions
- All packages build without errors
- Launch files are tested
- Documentation is updated

## Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [MyCobot Official Site](https://www.elephantrobotics.com/en/mycobot-en/)

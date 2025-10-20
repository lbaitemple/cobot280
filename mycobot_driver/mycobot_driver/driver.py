#!/usr/bin/env python3
'''
Copyright (c) 2021, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from mycobot.mycobot import MyCobot


class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver')
        
        self.lock = Lock()

        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 1000000)
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        self.get_logger().info(f'Connecting to MyCobot on {serial_port} at {baud_rate} baud')
        
        # Initialize robot
        self.robot = MyCobot(serial_port, str(baud_rate))
        self.robot.power_on()
        self.robot.set_pwm_mode(22, 0)
        self.robot.set_pwm_output(0, 0)

        # Subscribers
        self.gripper_sub = self.create_subscription(
            Float32,
            'gripper',
            self.gripper_cmd_callback,
            10
        )
        
        self.moveit_sub = self.create_subscription(
            JointState,
            'move_group/fake_controller_joint_states',
            self.joints_cmd_callback,
            10
        )
        
        self.gui_sub = self.create_subscription(
            JointState,
            'joints_gui',
            self.joints_gui_callback,
            10
        )
        
        # Publisher
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Joint configuration
        self.joint_names = [
            'joint2_to_joint1', 
            'joint3_to_joint2', 
            'joint4_to_joint3', 
            'joint5_to_joint4', 
            'joint6_to_joint5', 
            'joint6output_to_joint6'
        ]

        self.joints_cmd = [0, 0, 0, 0, 0, 0]
        self.joint_states = [0, 0, 0, 0, 0, 0]
        self.gripper_cmd = 0.018
        self.gripper_cmd_ack = True

        # Initialize joint state message
        self.joint_states_msg = JointState()
        self.joint_states_msg.header.frame_id = 'joint1'
        self.joint_states_msg.name = self.joint_names + ['gripper_base_to_l_finger']
        self.joint_states_msg.position = self.joint_states + [self.gripper_cmd]

        self.on_moveit = False

        # Create timer for control loop (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info('MyCobot driver started')

    def control_loop(self):
        """Main control loop called by timer"""
        # Get current joint angles from robot
        cur_actuator_angle = self.robot.get_radians()
        if len(cur_actuator_angle) == 6:
            self.joint_states = self.invert_joints(cur_actuator_angle)
        
        # Send commands to robot
        self.robot.send_radians(self.joints_cmd, 80)

        # Handle gripper command
        if not self.gripper_cmd_ack:
            self.gripper_cmd_ack = True
            pwm = self.dist_to_pwm(self.gripper_cmd)
            self.robot.set_pwm_output(0, pwm)

        # Publish joint states
        self.joint_states_msg.position[0:6] = self.joint_states
        self.joint_states_msg.position[6] = self.gripper_cmd
        self.joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_pub.publish(self.joint_states_msg)

    def gripper_cmd_callback(self, msg):
        """Handle gripper command"""
        with self.lock:
            self.gripper_cmd_ack = False
            self.gripper_cmd = msg.data

    def joints_cmd_callback(self, msg):
        """Handle joint commands from MoveIt"""
        with self.lock:
            self.on_moveit = True
            for joint_name, joint_position in zip(msg.name, msg.position):
                try:
                    j_idx = self.joint_names.index(joint_name)
                except ValueError:
                    continue

                invert = -1
                if j_idx == 2:
                    invert = 1

                self.joints_cmd[j_idx] = joint_position * invert

    def joints_gui_callback(self, msg):
        """Handle joint commands from GUI"""
        if self.on_moveit:
            return
            
        with self.lock:
            for joint_name, joint_position in zip(msg.name, msg.position):
                if joint_name == 'gripper_base_to_l_finger':
                    self.gripper_cmd_ack = False
                    self.gripper_cmd = joint_position
                    continue
                    
                try:
                    j_idx = self.joint_names.index(joint_name)
                except ValueError:
                    continue
                    
                self.joints_cmd[j_idx] = joint_position

    def invert_joints(self, joints):
        """Invert joint angles based on robot configuration"""
        joints = list(joints)  # Make a copy
        for i in range(len(joints)):
            if i != 2:
                joints[i] *= -1.0
        
        return joints

    def dist_to_pwm(self, dist):
        """Convert distance to PWM value for gripper"""
        return 200 - int((dist / 0.018) * 200)


def main(args=None):
    rclpy.init(args=args)
    
    driver = MyCobotDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# original imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# additional imports
from simple_pid import PID
from gokart_cmd_msgs.msg import BrakeCmd
from gokart_interface_msgs.msg.mcdaq import DAQain, DataAcquisition

import socket
import json


UDP_IP = "10.253.162.52"
UDP_PORT = 5005


class MinimalPublisher(Node):

    def __init__(self):             # TODO: add 3 parameters for tuning
        super().__init__('brake_pressure_control_node')
        # pub sub
        self.brake_cmd_pub = self.create_publisher(BrakeCmd, '~/brake_cmd', 5)

        # defining the PID controller
        self.keyboard_pressure = 0                                 
        # format of the arguments: (P, I, D, setpoint)
        self.pid = PID(2, 1, 2.5, self.keyboard_pressure)
        self.pid.output_limits = (0,1)

        self.pressure_sub = self.create_subscription(DataAcquisition, '~/mcdaq', self.listener_cb, 5)
        self.pressure_sub

        # UPD socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # TODO: one the other end of this socket,
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # map keys to different pressures using pygame
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
            self.sock.settimeout(0.05)
        except:
            print("-" * 20)

        self.brake = 0.0      # [0.0, 1,0]

        self.counter = 0  # keep 
        self.shutdown_in_progress = False

        # timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def listener_cb(self,msg):   # TODO: filtering of the pressure. SciPy Signal      
        if self.keyboard_pressure != 0:
            self.pid.setpoint = self.keyboard_pressure
            self.brake = self.pid(msg.ain.brake_pressure_right) # TODO: what about left/right?
        else:
            # if no key is pressed, we don't brake. Is this safe?
            self.brake = 0

        self.pub_cmds()

        
    # publishing the brake command msg
    def pub_cmds(self):
        timestamp = self.get_clock().now().to_msg()
        brake_cmd_msg = BrakeCmd()
        brake_cmd_msg.header.stamp = timestamp
        brake_cmd_msg.position = self.brake
        self.brake_cmd_pub.publish(brake_cmd_msg)   
    
    def cleanup(self):
        self.brake = 0.0
        self.pub_cmds()

    def react_no_valid_data(self):      # if no valid data is coming
        self.counter += 1

        if self.counter > 50:           # brake if no more data comes 
            self.brake = 1.0
            self.pub_cmds()

    # we dont need this cb fct if this node is also a subscriber, correct?
    def timer_cb(self):
        # self.get_logger().info(f"Hi{}")

        if self.shutdown_in_progress:
            self.cleanup()
            return
        
        try: 
            # acquiring the brake command
            data, addr = self.sock.revfrom(1024)
            data = data.decode()
            lst = json.loads(data)
            _, _, self.keyboard_pressure = map(float,lst)

            self.get_logger().info(f"{self.keyboard_pressure}")

            self.counter = 0
        except KeyboardInterrupt:
            self.shutdown_in_progress = True
            self.get_logger().info(f"No valid data. Counter: [{self.counter}]")
            self.react_no_valid_data()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

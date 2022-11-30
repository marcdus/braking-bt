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

# imports
import rclpy
import numpy as np
import scipy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

from simple_pid import PID
from gokart_cmd_msgs.msg import BrakeCmd
from gokart_interface_msgs.msg.mcdaq import DAQain, DataAcquisition
from filters.sos_filter import LiveSosFilter

import socket
import json


UDP_IP = "10.253.162.52"
UDP_PORT = 5005

class MinimalPublisher(Node):

    def __init__(self) -> None:
        super().__init__('brake_pressure_control_node')

        # for the numerical parameters we use an array
        self.params = np.array_([1.0, 1.0, 1.0, 1.0, 0.0], dtype='f')
        desc_params = ParameterDescriptor(description="set PID parameters as float array: [P, I, D, upper limit, default setpoint]")
        self.declare_parameter('params', self.params, desc_params)

        sos = scipy.signal.iirfilter(1, Wn=3, fs=20, btype="low",   # create second order section filter
                                    ftype="butter", output="sos")
        self.sosfilter = LiveSosFilter(sos)

        # OR we could keep the parameters seperate
        # self.tunings = (1, 1, 1)
        # desc_tunings = ParameterDescriptor(description='Set the P, I, D tuning values')
        # self.declare_parameter('tunings', self.tunings, desc_tunings)

        # self.lim_upper = 1.0
        # desc_lim_upper = ParameterDescriptor(description='Limit upper controller output (should be <= 1)')
        # self.declare_parameter('lim_upper', self.lim_upper, desc_lim_upper)

        # self.setpoint = 1.0
        # desc_setpoint = ParameterDescriptor(description='Choose the default setpoint to be tracked [bar]')
        # self.declare_parameter('setpoint', self.setpoint, desc_setpoint)

        # self.lim_lower = 0.0

        self.auto_mode = True
        desc_auto_mode = ParameterDescriptor(description='Enable/Disable the controller')
        self.declare_parameter('auto_mode', self.auto_mode, desc_auto_mode)

        self.prop_on_measurement = False
        desc_prop_on_measurement = ParameterDescriptor(description=
            'Enable/Disable calculating proportional term on measurement instead of error')
        self.declare_parameter('prop_on_measurement', self.prop_on_measurement, desc_prop_on_measurement)

        # declaring the pid controller option 2
        # self.pid = PID()
        # self.pid.tunings = self.tunings
        # self.pid.setpoint = self.setpoint
        # self.pid.output_limits = (self.lim_lower, self.lim_upper)
        # self.pid.auto_mode = self.auto_mode
        # self.pid.proportional_on_measurement = self.prop_on_measurement

        # declaring the pid controller
        self.pid = PID()
        self.pid.tunings = (self.params[0], self.params[1], self.params[2])
        self.pid.output_limits = (self.lim_lower, self.params[3])
        self.pid.setpoint = self.params[4]
        self.pid.auto_mode = self.auto_mode
        self.pid.proportional_on_measurement = self.prop_on_measurement
        

        # publish, subscribe
        self.brake_cmd_pub = self.create_publisher(BrakeCmd, '~/brake_cmd', 5)
        self.pressure_sub = self.create_subscription(DataAcquisition, '~/mcdaq', self.listener_cb, 5)
        self.pressure_sub

        # UPD socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # TODO: on the other end of this socket,
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

    def listener_cb(self,msg):
        if self.keyboard_pressure != 0:
            self.pid.setpoint = self.keyboard_pressure          # update setpoint
            pressure_input = max(msg.ain.brake_pressure_left,   # choose higher pressure as input
                                msg.ain.brake_pressure_right)
            pressure_filtered = self.sosfilter(pressure_input)  # filter
            self.brake = self.pid(pressure_filtered)            # apply the pid controller
        else:
            self.brake = 0      # if no key is pressed, we don't brake

        self.pub_cmds()         
        self.update_params()    #updating the pid parameters
    
    def update_params(self):
        # # option 1
        # self.tunings = self.get_parameter('tunings').get_parameter_value().THISVALUE # This line needs to be fixed
        # self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_value
        # self.lim_upper = self.get_parameter('lim_upper').get_parameter_value().double_value
        # self.auto_mode = self.get_parameter('auto_mode').get_parameter_value().bool_value
        # self.proportional_on_measurement = self.get_parameter('prop_on_measurement').get_parameter_value().bool_value
        update = False
        if self.params != self.get_parameter('params').get_parameter_value().double_array_value:
            update = True
            self.params = self.get_parameter('params').get_parameter_value().double_array_value
            self.pid.tunings = (self.params[0], self.params[1], self.params[2])
            self.pid.output_limits = (self.lim_lower, self.params[3])
            self.pid.setpoint = self.params[4]

        if self.auto_mode != self.get_parameter('auto_mode').get_parameter_value().bool_value:
            update = True
            self.auto_mode = self.get_parameter('auto_mode').get_parameter_value().bool_value
            self.pid.auto_mode = self.auto_mode
        
        if self.prop_on_measurement != self.get_parameter('prop_on_measurement').get_parameter_value().bool_value:
            update = True
            self.prop_on_measurement != self.get_parameter('prop_on_measurement').get_parameter_value().bool_value
            self.pid.proportional_on_measurement = self.prop_on_measurement
        
        # notify that pid params have been updated
        if update:
            self.get_logger().info(f"PID control parameters have been updated")
        

        
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

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


import rclpy
import numpy as np
import scipy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

from simple_pid import PID
from gokart_cmd_msgs.msg import BrakeCmd
from gokart_interface_msgs.msg import DAQain, DataAcquisition

from rclpy import qos

import socket


UDP_IP = "10.253.162.49"
UDP_PORT = 5005


class LiveFilter:
    def process(self, x):
        if np.isnan(x):     # skip value if it is invalid
            return(x)
        else:
            return self._process(x)
    
    def __call__(self, x):
        return self.process(x)
    
    def _process(self, x):      # for use if we want to try other filters
        raise NotImplementedError("derived processing class is missing")


class LiveSosFilter(LiveFilter):
    def __init__(self, sos):
        self.sos = sos

        self.n_sections = sos.shape[0]              # find out the shape of the second order section filter
        self.state = np.zeros((self.n_sections, 2)) # create empty "filter template"
    
    def _process(self, x):
        for s in range(self.n_sections):
            b0, b1, b2, a0, a1, a2 = self.sos[s, :]
            y = b0*x + self.state[s, 0]
            self.state[s, 0] = b1*x - a1*y + self.state[s, 1]
            self.state[s, 1] = b2*x - a2*y
            x = y
        return y


class MinimalPublisher(Node):

    def __init__(self) -> None:
        super().__init__('brake_pressure_control_node')

        # for the numerical parameters we use an array
        self.pid_params = [3.0, 0.15, 1.5]
        desc_pid_params = ParameterDescriptor(description='set PID parameters as float array: "[P, I, D"')
        self.declare_parameter('pid_params', self.pid_params, desc_pid_params)

        self.pid_limits = [-2000.0, 2000.0]
        desc_pid_limits = ParameterDescriptor(description='set PID output limits as float array: "[upper, lower]')
        self.declare_parameter('pid_limits', self.pid_limits, desc_pid_limits)

        self.auto_mode = True
        desc_auto_mode = ParameterDescriptor(description='Enable/Disable the controller')
        self.declare_parameter('auto_mode', self.auto_mode, desc_auto_mode)

        self.prop_on_measurement = False
        desc_prop_on_measurement = ParameterDescriptor(description=
            'Enable/Disable calculating proportional term on measurement instead of error')
        self.declare_parameter('prop_on_measurement', self.prop_on_measurement, desc_prop_on_measurement)
        
        self.filter_params = [1.0, 3.0, 20.0]
        desc_filter_params = ParameterDescriptor(description='set lowpass filter parameters as float array: "[n, f_stop, f_s]"')
        self.declare_parameter('filter_params', self.filter_params, desc_filter_params)
        
        # declare the filter
        
        sos = scipy.signal.iirfilter(self.filter_params[0], Wn=self.filter_params[1], fs=self.filter_params[2], btype="low",   # create second order section filter
                                    ftype="butter", output="sos")
        self.sosfilter = LiveSosFilter(sos)

        # declare the pid controller
        self.pid = PID()
        self.pid.tunings = (self.pid_params[0], self.pid_params[1], self.pid_params[2])
        self.pid.output_limits = (self.pid_limits[0], self.pid_limits[1])
        self.pid.auto_mode = self.auto_mode
        self.pid.proportional_on_measurement = self.prop_on_measurement

        # publish, subscribe
        self.brake_cmd_pub = self.create_publisher(BrakeCmd, '~/brake_cmd', 5)
        self.pressure_sub = self.create_subscription(DataAcquisition, '~/mcdaq', self.listener_cb, qos.qos_profile_sensor_data)
        self.pressure_sub

        # UPD socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # map keys to different pressures using pygame
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
            self.sock.settimeout(0.05)
        except:
            print("-" * 20)

        self.brake = 0.0      # [0.0, 1,0]
        self.keyboard_pressure = 0.0

        self.counter = 0  # keep 
        self.shutdown_in_progress = False

        # timer
        timer_period = 0.015
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def listener_cb(self,msg):
        if abs(self.keyboard_pressure) > 1e-7:  # significant non-zero value
            self.pid.setpoint = self.keyboard_pressure          # update setpoint
            pressure_input = max(msg.ain.brake_pressure_left,   # choose higher pressure as input
                                msg.ain.brake_pressure_right)
            pressure_filtered = self.sosfilter(pressure_input)  # filter
            error = self.keyboard_pressure - pressure_filtered
            
            self.brake += float(self.pid(pressure_filtered))/1000            # apply the pid controller

            self.brake = np.clip(self.brake, 0.0, 1.0)
            # self.brake = round(self.brake, 3)
            
        else:
            self.brake = 0.0      # if no key is pressed, we don't brake

        self.pub_cmds()         
        self.update_params()    #updating the pid parameters
    
    def update_params(self):
        update = False
        if self.pid_params != self.get_parameter('pid_params').get_parameter_value().double_array_value:
            update = True
            self.pid_params = self.get_parameter('pid_params').get_parameter_value().double_array_value
            self.pid.tunings = (self.pid_params[0], self.pid_params[1], self.pid_params[2])
            
        if self.pid_limits != self.get_parameter('pid_limits').get_parameter_value().double_array_value:
            update = True
            self.pid_limits = self.get_parameter('pid_limits').get_parameter_value().double_array_value
            self.pid.output_limits = (self.pid_limits[0], self.pid_limits[1])

        if self.auto_mode != self.get_parameter('auto_mode').get_parameter_value().bool_value:
            update = True
            self.auto_mode = self.get_parameter('auto_mode').get_parameter_value().bool_value
            self.pid.auto_mode = self.auto_mode
        
        if self.prop_on_measurement != self.get_parameter('prop_on_measurement').get_parameter_value().bool_value:
            update = True
            self.prop_on_measurement = self.get_parameter('prop_on_measurement').get_parameter_value().bool_value
            self.pid.proportional_on_measurement = self.prop_on_measurement
            
        if self.filter_params != self.get_parameter('filter_params').get_parameter_value().double_array_value:
            update = True
            self.filter_params = self.get_parameter('filter_params').get_parameter_value().double_array_value
        
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
            self.brake = 0.0
            self.pub_cmds()

    def timer_cb(self):
        # self.get_logger().info(f"Hi{}")

        if self.shutdown_in_progress:
            self.cleanup()
            return
        
        try: 
            # acquiring the brake command
            data, addr = self.sock.recvfrom(1024)
            data = data.decode()
            self.keyboard_pressure = float(data)
            self.get_logger().info(f"Demanded pressure: {self.keyboard_pressure}")

            self.counter = 0
        except KeyboardInterrupt:
            self.shutdown_in_progress = True
            self.get_logger().info(f"No valid data. Counter: [{self.counter}]")
            self.react_no_valid_data()
        except (socket.timeout, Exception) as e:
            self.get_logger().info(f"No valid data. Counter: [{self.counter}]")
            self.react_no_valid_data()
    

# class RemoteManualAgent(Node):
#     def __init__(self):
#         super().__init__('remote_manual_agent_node')
#         # pub sub
#         self.brake_cmd_pub = self.create_publisher(BrakeCmd, '~/brake_cmd', 5)

#         # timer
#         timer_period = 0.03  # secs
#         self.timer = self.create_timer(timer_period, self.timer_cb)

#         # UDP socket
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         try:
#             self.sock.bind((UDP_IP, UDP_PORT))
#             self.sock.settimeout(0.05)
#         except:
#             print("-" * 20)

#         self.brake = 0.0      # [0.0, 1,0]

#         self.counter = 0  # keep 
#         self.shutdown_in_progress = False

#     def pub_cmds(self):
#         timestamp = self.get_clock().now().to_msg()

#         brake_cmd_msg = BrakeCmd()
#         brake_cmd_msg.header.stamp = timestamp 
#         brake_cmd_msg.position = self.brake
#         self.brake_cmd_pub.publish(brake_cmd_msg)

#     def cleanup(self):
#         # self.steer = 0.0
#         self.brake = 0.0
#         self.pub_cmds()

#     def react_no_valid_data(self):
#         # in case no valid data is coming
#         self.counter += 1

#         # number of cycles of timer_period
#         if self.counter > 50:
#             # brake, in case of e.g. loss of internet
#             self.brake = 0.0
#             self.throttle = 0.0
#             self.pub_cmds()
            
#     def timer_cb(self):
#         # self.get_logger().info(f"Hi{}")

#         if self.shutdown_in_progress:
#             self.cleanup()
#             return

#         # recv udp
#         try:
#             data, addr = self.sock.recvfrom(1024)
#             data = data.decode()
#             # print(data)
#             self.brake = float(data)
#             self.get_logger().info(f"Brake: [{self.brake}]")
#             self.pub_cmds()
#             self.counter = 0
#         except KeyboardInterrupt:
#             self.shutdown_in_progress = True
#             self.get_logger().info("[Ctrl+C] captured. Sending 0.0 to throttle and brake for 2 seconds")
#         except (socket.timeout, Exception) as e:
#             self.get_logger().info(f"No valid data. Counter: [{self.counter}]")
#             self.react_no_valid_data()

# def main(args=None):
#     rclpy.init(args=args)
#     remote_manual_agent = RemoteManualAgent()
#     rclpy.spin(remote_manual_agent)

#     remote_manual_agent.destroy_node()
#     rclpy.shutdown()

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

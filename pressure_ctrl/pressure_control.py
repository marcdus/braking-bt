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

        # declaring parameters
        self.pid_params = [3.0, 0.15, 1.5]
        desc_pid_params = ParameterDescriptor(description=
        'set PID parameters as float array: "[P, I, D"')
        self.declare_parameter('pid_params', self.pid_params, desc_pid_params)

        self.pid_limits = [-2000.0, 2000.0]
        desc_pid_limits = ParameterDescriptor(description=
        'set PID output limits as float array: "[upper, lower]')
        self.declare_parameter('pid_limits', self.pid_limits, desc_pid_limits)

        self.auto_mode = True
        desc_auto_mode = ParameterDescriptor(description=
        'Enable/Disable the controller')
        self.declare_parameter('auto_mode', self.auto_mode, desc_auto_mode)

        self.prop_on_measurement = False
        desc_prop_on_measurement = ParameterDescriptor(description=
            'Enable/Disable calculating proportional term on measurement instead of error')
        self.declare_parameter('prop_on_measurement', self.prop_on_measurement, desc_prop_on_measurement)
        
        self.filter_pressure_params = [1.0, 3.0, 20.0]
        desc_filter_pressure_params = ParameterDescriptor(
            description='set lowpass filter parameters as float array: "[n, f_stop, , on/off]"')
        self.declare_parameter('filter_pressure_params', self.filter_pressure_params, desc_filter_pressure_params)

        self.filter_pressure_toggle = True
        desc_filter_pressure_toggle = ParameterDescriptor(description='toggle pressure filter')
        self.declare_parameter('filter_pressure_toggle', self.filter_pressure_toggle, desc_filter_pressure_toggle)

        self.filter_output_params = [1.0, 5.0, 20.0]
        desc_filter_output_params = ParameterDescriptor(
            description='set lowpass output filter parameters as float array')
        self.declare_parameter('filter_output_params', self.filter_output_params, desc_filter_output_params)

        self.filter_output_toggle = True
        desc_filter_output_toggle = ParameterDescriptor(description='toggle output filter')
        self.declare_parameter('filter_output_toggle', self.filter_output_toggle, desc_filter_output_toggle)
        
        # declaring the filters
        self.update_filters()

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
            pressure_input = max(msg.ain.brake_pressure_left, msg.ain.brake_pressure_right)
            if self.filter_pressure_toggle:
                pressure_input = self.filter_pressure(pressure_input)   # filter the input

            self.brake += self.pid(pressure_input)/1000     # apply the pid controller
            if self.filter_output_toggle:
                self.brake = self.filter_output(self.brake) # filter the output

            self.brake = np.clip(self.brake, 0.0, 1.0)      # constrain output to the allowed range
        else:
            self.brake = 0.0    # if no key is pressed, we don't brake

        self.pub_cmds()         
        self.update_params()    # updating the pid parameters
    
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
            
        if self.filter_pressure_params != self.get_parameter('filter_pressure_params').get_parameter_value().double_array_value:
            update = True
            self.filter_pressure_params = self.get_parameter('filter_pressure_params').get_parameter_value().double_array_value
            self.update_filters()

        if self.filter_pressure_toggle != self.get_parameter('filter_pressure_toggle').get_parameter_value().bool_value:
            update = True
            self.filter_pressure_toggle = self.get_parameter('filter_pressure_toggle').get_parameter_value().bool_value

        if self.filter_output_params != self.get_parameter('filter_output_params').get_parameter_value().double_array_value:
            update = True
            self.filter_output_params = self.get_parameter('filter_output_params').get_parameter_value().double_array_value
            self.update_filters()

        if self.filter_output_toggle != self.get_parameter('filter_output_toggle').get_parameter_value().bool_value:
            update = True
            self.filter_output_toggle = self.get_parameter('filter_output_toggle').get_parameter_value().bool_value

        # notify that params have been updated
        if update:
            self.get_logger().info(f"parameter(s) updated")
    
    # declaring/updating the filters
    def update_filters(self):
        # pressure filter
        sos = scipy.signal.iirfilter(
            self.filter_pressure_params[0], 
            Wn=self.filter_pressure_params[1], 
            fs=self.filter_pressure_params[2], 
            btype="low", ftype="butter", output="sos")
        self.filter_pressure = LiveSosFilter(sos)

        # output filter
        sos = scipy.signal.iirfilter(
            self.filter_output_params[0], 
            Wn=self.filter_output_params[1],
            fs=self.filter_output_params[2],
            btype="low", ftype="butter", output="sos")
        self.filter_output = LiveSosFilter(sos)
        
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

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

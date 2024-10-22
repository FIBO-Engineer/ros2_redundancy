#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from tools_lib import *

class PrimaryNode(Node):
    def __init__(self):
        super().__init__('primary_node')
        self.declare_parameter('primary_ip', '192.168.127.103')  
        self.declare_parameter('primary_port', 1254) 
        self.declare_parameter('redundant_ip', '192.168.127.104')  
        self.declare_parameter('redundant_port', 1254) 
        self.declare_parameter('ros2_command', 'ros2 launch yamaha_ros2 bs_master.launch.py') 
        self.main_ip = self.get_parameter('primary_ip').get_parameter_value().string_value
        self.main_port = self.get_parameter('primary_port').get_parameter_value().integer_value
        self.redundant_ip = self.get_parameter('redundant_ip').get_parameter_value().string_value
        self.redundant_port = self.get_parameter('redundant_port').get_parameter_value().integer_value
        self.ros2_command = self.get_parameter('ros2_command').get_parameter_value().string_value
        self.status_pub = self.create_publisher(RedundancyStatus, 'redundancy_status', 10)
        self.sock = SocketToolsLib(self.main_ip, self.main_port, 0.5)
        self.is_main_initial = False
        self.main_status = b'\x01\x00'
        self.timestamp = 0
        self.primary_node_running = False
        self.primary_controller_active = False
        self.redundant_node_running = False
        self.redundant_controller_active = False
        self.safety_count = 0
        self.safety_count_desire = 5
        self.safety_count_2 = 0
        self.safety_count_desire_2 = 5

    def status_publisher(self):
        msg = RedundancyStatus()
        msg.primary_node_running = self.primary_node_running
        msg.primary_controller_active = self.primary_controller_active
        msg.redundant_node_running = self.redundant_node_running
        msg.redundant_controller_active = self.redundant_controller_active
        self.status_pub.publish(msg)
        # self.get_logger().info('Publish the message!')

    def primary_run(self):
        if not self.is_main_initial:
            self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
            receive_data = self.sock.receive()
            if receive_data[0] == b'\x02\x01' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:                
                self.safety_count = 0
                self.safety_count_2 = 0 
                if StateHandler.print_message(MainState.CASE1):
                    self.primary_node_running = False
                    self.primary_controller_active = False
                    self.redundant_node_running = True
                    self.redundant_controller_active = True
                    self.get_logger().info('Primary: Killing primary node, Redundant: Redundant node and app is running')
                self.status_publisher()
                self.destroy_node()
                rclpy.shutdown()
                return              
            elif receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                self.safety_count = 0
                self.safety_count_2 = 0
                if StateHandler.print_message(MainState.CASE2):
                    self.primary_node_running = True
                    self.primary_controller_active = True
                    self.redundant_node_running = True
                    self.redundant_controller_active = False 
                    self.get_logger().info('Primary: Primary node and app are running, Redundant: Redundant node is running but app is not run')
                self.main_status = b'\x01\x01'
                self.is_main_initial = True
                sub_process_function(self.ros2_command)
            elif receive_data[0] == 'No data received':
                self.safety_count_2 = 0 
                if self.safety_count == self.safety_count_desire:
                    if StateHandler.print_message(MainState.CASE3):
                        self.primary_node_running = True
                        self.primary_controller_active = True
                        self.redundant_node_running = False
                        self.redundant_controller_active = False 
                        self.get_logger().warn('Primary: Primary node and app are running, Redundant: Redundant node and app are not run')
                    self.main_status = b'\x01\x01'
                    self.is_main_initial = True
                    sub_process_function(self.ros2_command)
                    self.safety_count = 0
                elif self.safety_count < self.safety_count_desire:
                    self.safety_count += 1
            else:
                self.safety_count = 0  
                if self.safety_count_2 == self.safety_count_desire_2:
                    if StateHandler.print_message(MainState.CASE4):
                        self.primary_node_running = False
                        self.primary_controller_active = False
                        self.redundant_node_running = False
                        self.redundant_controller_active = False 
                        self.get_logger().error('Out of scope!')
                    self.main_status = b'\x01\x01'
                    self.is_main_initial = True   
                    sub_process_function(self.ros2_command) 
                    self.safety_count_2 = 0     
                elif self.safety_count_2 < self.safety_count_desire_2:  
                    self.safety_count_2 += 1
            self.status_publisher()
        else:
            if time.time() - self.timestamp > 0.2:
                self.timestamp = time.time()
                if is_command_running(self.ros2_command):
                    self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
                    receive_data = self.sock.receive()
                    if receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        if StateHandler.print_message(MainState.CASE5):
                            self.primary_node_running = True
                            self.primary_controller_active = True
                            self.redundant_node_running = True
                            self.redundant_controller_active = False 
                            self.get_logger().info('Primary: Primary node and app are running, Redundant: Redundant node is running but app is not run')
                    elif receive_data[0] == 'No data received':
                        if StateHandler.print_message(MainState.CASE6):
                            self.primary_node_running = True
                            self.primary_controller_active = True
                            self.redundant_node_running = False
                            self.redundant_controller_active = False 
                            self.get_logger().info('Primary: Primary node and app are running, Redundant: Redundant node and app are not run')                
                    else:
                        if StateHandler.print_message(MainState.CASE7):
                            self.primary_node_running = False
                            self.primary_controller_active = False
                            self.redundant_node_running = False
                            self.redundant_controller_active = False 
                            self.get_logger().error('Out of scope!')  
                else:
                    self.main_status = b'\x01\x02'
                    self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
                    receive_data = self.sock.receive()
                    if receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        if StateHandler.print_message(MainState.CASE8):
                            self.primary_node_running = True
                            self.primary_controller_active = False
                            self.redundant_node_running = True
                            self.redundant_controller_active = False 
                            self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node is running but app is not run')
                    elif receive_data[0] == b'\x02\x01' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        if StateHandler.print_message(MainState.CASE9):
                            self.primary_node_running = False
                            self.primary_controller_active = False
                            self.redundant_node_running = True
                            self.redundant_controller_active = True 
                            self.get_logger().info('Primary: Killing primary node, Redundant: Redundant node and app are running')
                        self.status_publisher()
                        self.destroy_node()
                        rclpy.shutdown()
                        return                        
                    elif receive_data[0] == 'No data received':
                        if StateHandler.print_message(MainState.CASE10):
                            self.primary_node_running = True
                            self.primary_controller_active = False
                            self.redundant_node_running = False
                            self.redundant_controller_active = False 
                            self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node and app are not run')
                    else:
                        if StateHandler.print_message(MainState.CASE11):
                            self.primary_node_running = False
                            self.primary_controller_active = False
                            self.redundant_node_running = False
                            self.redundant_controller_active = False 
                            self.get_logger().error('Out of scope!')
                self.status_publisher()

def primary(args=None):
    rclpy.init(args=args)
    primary_node = PrimaryNode()
    try:
        while rclpy.ok():
            primary_node.primary_run()
        primary_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        primary_node.get_logger().error(f'The node error is {e}')
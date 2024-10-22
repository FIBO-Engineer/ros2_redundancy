#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from tools_lib import *

class RedundantNode(Node):
    def __init__(self):
        super().__init__('redundant_node')
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
        self.is_redundant_initial = False
        self.sock = SocketToolsLib(self.redundant_ip, self.redundant_port, 0.5)
        self.redundant_status = b'\x02\x00'
        self.is_run_app = False
        self.primary_node_running = False
        self.primary_controller_active = False
        self.redundant_node_running = False
        self.redundant_controller_active = False
        self.safety_count = 0
        self.safety_count_desire = 5

    def status_publisher(self):
        msg = RedundancyStatus()
        msg.primary_node_running = self.primary_node_running
        msg.primary_controller_active = self.primary_controller_active
        msg.redundant_node_running = self.redundant_node_running
        msg.redundant_controller_active = self.redundant_controller_active
        self.status_pub.publish(msg)

    def redundant_run(self):
        if not self.is_redundant_initial:
            receive_data = self.sock.receive()
            if receive_data[0] == b'\x01\x00' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                time.sleep(0.05)
                self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                if StateHandler2.print_message(RedundantState.CASE1):    
                    self.primary_node_running = True
                    self.primary_controller_active = False
                    self.redundant_node_running = True
                    self.redundant_controller_active = False 
                    self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node is running but app is not run')
            elif receive_data[0] == b'\x01\x01' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                time.sleep(0.05)
                self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                self.is_redundant_initial = True
                if StateHandler2.print_message(RedundantState.CASE2):
                    self.primary_node_running = True
                    self.primary_controller_active = True
                    self.redundant_node_running = True
                    self.redundant_controller_active = False 
                    self.get_logger().info('Primary: Primary node and app are running, Redundant: Redundant node is running but app is not run')
            elif receive_data[0] == b'\x01\x02' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                time.sleep(0.05)
                self.redundant_status = b'\x02\x01'
                self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                if StateHandler2.print_message(RedundantState.CASE3):
                    self.primary_node_running = True
                    self.primary_controller_active = False
                    self.redundant_node_running = True
                    self.redundant_controller_active = True 
                    self.get_logger().info('Primary: Primary node is running but app is not run because error, Redundant: Redundant node and app are running')  
                self.is_redundant_initial = True
                self.is_run_app = True  
                sub_process_function(self.ros2_command)  
            elif receive_data[0] == 'No data received':
                time.sleep(0.05)
                self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                if StateHandler2.print_message(RedundantState.CASE4):
                    self.primary_node_running = False
                    self.primary_controller_active = False
                    self.redundant_node_running = True
                    self.redundant_controller_active = False 
                    self.get_logger().info('Primary: Primary node and app are not run, Redundant: Redundant node is running but app is not run')  
            else:
                time.sleep(0.05)
                self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                if StateHandler2.print_message(RedundantState.CASE5):
                    self.primary_node_running = False
                    self.primary_controller_active = False
                    self.redundant_node_running = False
                    self.redundant_controller_active = False 
                    self.get_logger().error('Out of scope!')  
            self.status_publisher()
        else:        
            receive_data = self.sock.receive()
            if self.is_run_app:
                if receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                    time.sleep(0.05)
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status)      
                    if StateHandler2.print_message(RedundantState.CASE6):
                        self.primary_node_running = True
                        self.primary_controller_active = False
                        self.redundant_node_running = True
                        self.redundant_controller_active = True 
                        self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node and app are running')               
                elif receive_data[0] == 'No data received':
                    time.sleep(0.05)
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status) 
                    if StateHandler2.print_message(RedundantState.CASE7):
                        self.primary_node_running = False
                        self.primary_controller_active = False
                        self.redundant_node_running = True
                        self.redundant_controller_active = True 
                        self.get_logger().info('Primary: Primary node and app are not run, Redundant: Redundant node and app are running')     
                else:
                    time.sleep(0.05)
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status) 
                    if StateHandler2.print_message(RedundantState.CASE8):
                        self.primary_node_running = False
                        self.primary_controller_active = False
                        self.redundant_node_running = False
                        self.redundant_controller_active = False 
                        self.get_logger().error('Out of scope!')    
            else:
                if receive_data[0] == b'\x01\x00' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                    self.safety_count = 0
                    time.sleep(0.05)
                    self.redundant_status = b'\x02\x01'
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                    if StateHandler2.print_message(RedundantState.CASE9):
                        self.primary_node_running = True
                        self.primary_controller_active = False
                        self.redundant_node_running = True
                        self.redundant_controller_active = True 
                        self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node and app are running')    
                    self.is_run_app = True  
                    sub_process_function(self.ros2_command) 
                elif receive_data[0] == b'\x01\x01' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                    self.safety_count = 0
                    time.sleep(0.05)
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                    if StateHandler2.print_message(RedundantState.CASE10):
                        self.primary_node_running = True
                        self.primary_controller_active = True
                        self.redundant_node_running = True
                        self.redundant_controller_active = False 
                        self.get_logger().info('Primary: Primary node and app are running, Redundant: Redundant node is running but app is not run')    
                elif receive_data[0] == b'\x01\x02' and receive_data[1] == self.main_ip and receive_data[2] == self.main_port:
                    self.safety_count = 0
                    time.sleep(0.05)
                    self.redundant_status = b'\x02\x01'
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                    if StateHandler2.print_message(RedundantState.CASE11):
                        self.primary_node_running = True
                        self.primary_controller_active = False
                        self.redundant_node_running = True
                        self.redundant_controller_active = True 
                        self.get_logger().info('Primary: Primary node is running but app is not run, Redundant: Redundant node and app are running') 
                    self.is_run_app = True  
                    sub_process_function(self.ros2_command) 
                elif receive_data[0] == 'No data received':
                    time.sleep(0.05)
                    if self.safety_count == self.safety_count_desire:
                        self.redundant_status = b'\x02\x01'
                        self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                        if StateHandler2.print_message(RedundantState.CASE12):
                            self.primary_node_running = False
                            self.primary_controller_active = False
                            self.redundant_node_running = True
                            self.redundant_controller_active = True 
                            self.get_logger().info('Primary: Primary node and app are not run, Redundant: Redundant node and app are running') 
                        self.is_run_app = True  
                        sub_process_function(self.ros2_command)
                        self.safety_count = 0
                    elif self.safety_count < self.safety_count_desire:
                        self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                        self.safety_count += 1
                    else:
                        self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                else:
                    self.safety_count = 0
                    time.sleep(0.05)
                    self.sock.send(self.main_ip, self.main_port, self.redundant_status)
                    # self.get_logger().info('14')   
                    if StateHandler2.print_message(RedundantState.CASE13):
                        self.primary_node_running = False
                        self.primary_controller_active = False
                        self.redundant_node_running = False
                        self.redundant_controller_active = False 
                        self.get_logger().error('Out of scope!')  
            self.status_publisher()                  

def redundant(args=None):
    rclpy.init(args=args)
    redundant_node = RedundantNode()
    try:
        while rclpy.ok():
            redundant_node.redundant_run()
        redundant_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        redundant_node.get_logger().error(f'The node error is {e}')
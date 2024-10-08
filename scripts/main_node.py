#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from tools_lib import *

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.declare_parameter('main_ip', '192.168.127.103')  
        self.declare_parameter('main_port', 1254) 
        self.declare_parameter('redundant_ip', '192.168.127.104')  
        self.declare_parameter('redundant_port', 1254) 
        self.declare_parameter('ros2_command', 'ros2 run demo_nodes_cpp talker') 
        self.main_ip = self.get_parameter('main_ip').get_parameter_value().string_value
        self.main_port = self.get_parameter('main_port').get_parameter_value().integer_value
        self.redundant_ip = self.get_parameter('redundant_ip').get_parameter_value().string_value
        self.redundant_port = self.get_parameter('redundant_port').get_parameter_value().integer_value
        self.ros2_command = self.get_parameter('ros2_command').get_parameter_value().string_value
        self.status_pub = self.create_publisher(Status, 'status', 10)
        self.sock = SocketToolsLib(self.main_ip, self.main_port, 0.005)
        self.is_main_initial = False
        self.main_status = b'\x01\x00'
        self.timestamp = 0
        self.is_main_node_run = False
        self.is_app_run_in_main_node = False
        self.is_redundant_node_run = False
        self.is_app_run_in_redundant_node = False

    def status_publisher(self):
        msg = Status()
        msg.is_main_node_run = self.is_main_node_run
        msg.is_app_run_in_main_node = self.is_app_run_in_main_node
        msg.is_redundant_node_run = self.is_redundant_node_run
        msg.is_app_run_in_redundant_node = self.is_app_run_in_redundant_node
        self.status_pub.publish(msg)
        self.get_logger().info('Publish the message!')

    def main_run(self):
        if not self.is_main_initial:
            self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
            receive_data = self.sock.receive()
            if receive_data[0] == b'\x02\x01' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                self.get_logger().info('Main: Killing main node, Redundant: Redundant node and app is running')
                self.is_main_node_run = False
                self.is_app_run_in_main_node = False
                self.is_redundant_node_run = True
                self.is_app_run_in_redundant_node = True
                self.destroy_node()
                rclpy.shutdown()
                return                
            elif receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                self.get_logger().info('Main: Main node and app are running, Redundant: Redundant node is running but app is not run')
                sub_process_function(self.ros2_command)
                self.main_status = b'\x01\x01'
                self.is_main_initial = True
                self.is_main_node_run = True
                self.is_app_run_in_main_node = True
                self.is_redundant_node_run = True
                self.is_app_run_in_redundant_node = False
            elif receive_data[0] == 'No data received':
                self.get_logger().warn('Main: Main node and app are running, Redundant: Redundant node and app are not run')
                sub_process_function(self.ros2_command)
                self.main_status = b'\x01\x01'
                self.is_main_initial = True
                self.is_main_node_run = True
                self.is_app_run_in_main_node = True
                self.is_redundant_node_run = False
                self.is_app_run_in_redundant_node = False
            else:
                self.get_logger().error('Out of scope!')
                self.is_main_node_run = False
                self.is_app_run_in_main_node = False
                self.is_redundant_node_run = False
                self.is_app_run_in_redundant_node = False
                self.destroy_node()
                rclpy.shutdown()
                return                
        else:
            if time.time() - self.timestamp > 0.002:
                self.timestamp = time.time()
                if is_command_running(self.ros2_command):
                    self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
                    receive_data = self.sock.receive()
                    if receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        self.get_logger().info('Main: Main node and app are running, Redundant: Redundant node is running but app is not run')                    
                        self.is_main_node_run = True
                        self.is_app_run_in_main_node = True
                        self.is_redundant_node_run = True
                        self.is_app_run_in_redundant_node = False
                    elif receive_data[0] == 'No data received':
                        self.get_logger().info('Main: Main node and app are running, Redundant: Redundant node and app are not run')
                        self.is_main_node_run = True
                        self.is_app_run_in_main_node = True
                        self.is_redundant_node_run = False
                        self.is_app_run_in_redundant_node = False                    
                    else:
                        self.get_logger().error('Out of scope!')
                        self.is_main_node_run = False
                        self.is_app_run_in_main_node = False
                        self.is_redundant_node_run = False
                        self.is_app_run_in_redundant_node = False
                        self.destroy_node()
                        rclpy.shutdown()
                        return   
                else:
                    self.main_status = b'\x01\x02'
                    self.sock.send(self.redundant_ip, self.redundant_port , self.main_status)
                    receive_data = self.sock.receive()
                    if receive_data[0] == b'\x02\x00' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        self.get_logger().error('Main: Main node and app are not run and error, Redundant: Redundant node is running but app is not run')
                        self.is_main_node_run = False
                        self.is_app_run_in_main_node = False
                        self.is_redundant_node_run = True
                        self.is_app_run_in_redundant_node = False
                    elif receive_data[0] == b'\x02\x01' and receive_data[1] == self.redundant_ip and receive_data[2] == self.redundant_port:
                        self.get_logger().info('Main: Main node and app are not run and error and Killing main node, Redundant: Redundant node and app are running')
                        self.is_main_node_run = False
                        self.is_app_run_in_main_node = False
                        self.is_redundant_node_run = True
                        self.is_app_run_in_redundant_node = True
                        self.destroy_node()
                        rclpy.shutdown()
                        return
                    elif receive_data[0] == 'No data received':
                        self.get_logger().error('Main: Main node and app are not run and error, Redundant: Redundant node and app are not run')
                        self.is_main_node_run = False
                        self.is_app_run_in_main_node = False
                        self.is_redundant_node_run = False
                        self.is_app_run_in_redundant_node = False
                    else:
                        self.get_logger().error('Out of scope!')
                        self.is_main_node_run = False
                        self.is_app_run_in_main_node = False
                        self.is_redundant_node_run = False
                        self.is_app_run_in_redundant_node = False
                        self.destroy_node()
                        rclpy.shutdown()
                        return
                self.status_publisher()

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    try:
        while rclpy.ok():
            main_node.main_run()
        main_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        main_node.get_logger().error(f'The node error is {e}')
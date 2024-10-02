#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import socket
import psutil
import time
from ros2_redundancy.msg import Status

def sub_process_function(command):
    subprocess.Popen(command.split())
    
def is_command_running(command):
    command_list = command.split()
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if command_list[0] == "ros2":
                if ('/usr/bin/python3' in proc.info['cmdline'] and 
                    '/opt/ros/humble/bin/ros2' in proc.info['cmdline'] and
                    command_list[1] in proc.info['cmdline'] and  
                    command_list[2] in proc.info['cmdline'] and 
                    command_list[3] in proc.info['cmdline']):
                    return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False

class SocketToolsLib():
    def __init__(self, host_ip, host_port, timeout):
        self.socket_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_ip = host_ip
        self.host_port = host_port 
        self.socket_s.bind((self.host_ip, self.host_port))
        self.socket_s.settimeout(timeout)

    def send(self, server_ip, server_port, send_data):
        self.socket_s.sendto(send_data, (server_ip, server_port))

    def receive(self):
        try:
            data, address = self.socket_s.recvfrom(1024)
            receive_data = [data, address[0], address[1]]
        except:
            receive_data = ["No data received", "", 0]
        return receive_data    
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import socket
import psutil
import time
from ros2_redundancy.msg import RedundancyStatus
from enum import Enum

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

class MainState(Enum):
    CASE1 = 1
    CASE2 = 2
    CASE3 = 3
    CASE4 = 4
    CASE5 = 5
    CASE6 = 6
    CASE7 = 7
    CASE8 = 8
    CASE9 = 9
    CASE10 = 10
    CASE11 = 11

class StateHandler:
    printed_cases = set()
    last_case = None

    @staticmethod
    def print_message(state):
        if StateHandler.last_case != state:
            StateHandler.printed_cases.clear()
        if state not in StateHandler.printed_cases:
            StateHandler.printed_cases.add(state)
            StateHandler.last_case = state
            return True
        StateHandler.last_case = state
        return False

class RedundantState(Enum):
    CASE1 = 1
    CASE2 = 2
    CASE3 = 3
    CASE4 = 4
    CASE5 = 5
    CASE6 = 6
    CASE7 = 7
    CASE8 = 8
    CASE9 = 9
    CASE10 = 10
    CASE11 = 11
    CASE12 = 12
    CASE13 = 13

class StateHandler2:
    printed_cases_2 = set()
    last_case_2 = None

    @staticmethod
    def print_message(state):
        if StateHandler2.last_case_2 != state:
            StateHandler2.printed_cases_2.clear()
        if state not in StateHandler2.printed_cases_2:
            StateHandler2.printed_cases_2.add(state)
            StateHandler2.last_case_2 = state
            return True
        StateHandler2.last_case_2 = state
        return False

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_redundancy',
            executable='primary.py',
            name='primary_node',
            parameters=[
                {
                    'primary_ip': '192.168.127.103'
                },{
                    'primary_port': 1254
                },{
                    'redundant_ip': '192.168.127.104'
                },{
                    'redundant_port': 1254
                },{
                    'ros2_command': 'ros2 launch yamaha_ros2 bs_master.launch.py'
                }
            ],
            output='screen'
        ),
    ])

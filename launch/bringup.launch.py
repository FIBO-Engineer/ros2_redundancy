from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_redundancy',
            executable='redundant.py',
            name='redundant_node',
            parameters=[
                {
                    'main_ip': '192.168.100.5'
                },{
                    'main_port': 1254
                },{
                    'redundant_ip': '192.168.100.4'
                },{
                    'redundant_port': 1254
                },{
                    'ros2_command': 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
                }
            ],
            output='screen'
        ),
    ])

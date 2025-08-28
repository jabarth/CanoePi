from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the CanoeDash Zero Host.

    This launch file starts two nodes:
    1. The micro-ROS agent to communicate with the Pico.
    2. The BLE-to-ROS bridge node to communicate with the Android app.
    """
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyAMA0', '-b', '115200']
        ),
        Node(
            package='ble_ros_bridge',
            executable='ble_ros_bridge_node',
            name='ble_ros_bridge_node',
            output='screen'
        ),
    ])

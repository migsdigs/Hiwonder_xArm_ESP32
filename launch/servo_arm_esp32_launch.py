from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB0", "-v6"]
        ),
        Node(
            package='arm_servos_pubs_subs',
            executable='talker',
            name='move_servos_publisher',
            # output='screen',
        ),
        Node(
            package='arm_servos_pubs_subs',
            executable='listener',
            name='servo_pos_subscriber',
            # output='screen',
        ),
    ])

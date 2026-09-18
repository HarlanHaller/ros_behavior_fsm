from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONUNBUFFERED',
            value='1'
        ),
        Node(
            executable='attack',
            package='ros_behavior_fsm',
            output='screen',
            # env={'PYTHONUNBUFFERED': '1'}
        ),
        Node(
            executable='patrol',
            package='ros_behavior_fsm',
            output='screen',
            # env={'PYTHONUNBUFFERED': '1'}
        ),
        Node(
            executable='suspicious',
            package='ros_behavior_fsm',
            output='screen',
            # env={'PYTHONUNBUFFERED': '1'}
        ),
        Node(
            executable='object_detection',
            package='ros_behavior_fsm'
        )
    ])
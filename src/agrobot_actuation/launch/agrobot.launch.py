from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agrobot_actuation',
            executable='remove',
            name='remove',
            output='screen'
        ),
        Node(
            package='agrobot_actuation',
            executable='seed',
            name='seed',
            output='screen'
        ),
        Node(
            package='agrobot_actuation',
            executable='button_client',
            name='button_client',
            output='screen'
        ),
    ])

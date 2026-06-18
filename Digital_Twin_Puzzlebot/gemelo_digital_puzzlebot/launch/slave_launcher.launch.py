from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='gemelo_digital_puzzlebot',
            executable='obstacle_detector.py',
            output='screen'
        )
    ])
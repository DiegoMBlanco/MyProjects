from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='gemelo_digital_puzzlebot',
            executable='line_detector.py',
            name='line_detector',
            output='screen'
        ),

        Node(
            package='gemelo_digital_puzzlebot',
            executable='intersection.py',
            output='screen'
        ),

        Node(
            package='gemelo_digital_puzzlebot',
            executable='obstacle_detector.py',
            output='screen'
        ),

        Node(
            package='gemelo_digital_puzzlebot',
            executable='line_follower.py',
            name='pid_controller',
            output='screen'
        )

    ])

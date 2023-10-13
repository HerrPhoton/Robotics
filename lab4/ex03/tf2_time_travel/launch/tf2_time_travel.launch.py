from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='tf2_time_travel',
            executable='tf2_frame_publisher',
            name='publisher1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='tf2_time_travel',
            executable='tf2_frame_publisher',
            name='publisher2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='tf2_time_travel',
            executable='tf2_listener',
            name='listener',
            parameters=[{
                'target_frame': 'turtle1',
                'delay': LaunchConfiguration('delay', default = 1.0),
                }],
        ),
    ])
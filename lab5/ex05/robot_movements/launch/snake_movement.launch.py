import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_project_description = get_package_share_directory('diff_drive_robot')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_description, 'launch', 'robot_gazebo.launch.py')))


    return LaunchDescription([
        gz_sim,

        Node(
            package='robot_movements',
            executable='snake_movement',
            name='movement'),

    ])

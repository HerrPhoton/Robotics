import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_description = get_package_share_directory('diff_drive_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_lidar = LaunchConfiguration('use_lidar', default = 'true')
    use_camera = LaunchConfiguration('use_camera', default = 'true')

    urdf_path = os.path.join(pkg_project_description, 'urdf', 'tank.urdf.xacro')
    robot_desc = ParameterValue(
        Command(['xacro ', urdf_path, ' use_lidar:=', use_lidar, ' use_camera:=', use_camera]), value_type = str)

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments = {'gz_args': LaunchConfiguration('world', default = '-r empty.sdf')}.items(),
    )

    # Spawn robot
    create = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = ['-name', 'robot',
                     '-topic', 'robot_description',
                     '-x', '0.0',
                     '-y', '0.0',
                     '-z', '0.3',
                     ],
        output = 'screen',
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'both',
        parameters = [
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"}
        ]
    )

    # Visualize in RViz
    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = [
            '-d', os.path.join(pkg_project_description, 'rviz', 'diff_drive.rviz')],
            condition = IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        parameters = [{
            'config_file': os.path.join(pkg_project_description, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output = 'screen'
    )
    

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value = 'true', description = 'Open RViz.'),
        gz_sim,
        bridge,
        robot_state_publisher,
        rviz,
        TimerAction(
            period = 5.0,
            actions = [create])
        ])

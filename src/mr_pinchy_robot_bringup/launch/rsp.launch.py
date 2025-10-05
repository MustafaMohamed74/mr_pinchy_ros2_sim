from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ['true', '1', 'yes']
    urdf = LaunchConfiguration('urdf').perform(context)

    # Robot description from xacro
    robot_description = {
        'robot_description': Command(['xacro ', urdf])
    }

    # Load controller config
    controller_yaml_path = os.path.join(
        get_package_share_directory("mr_pinchy_robot_bringup"),
        "config",
        "controllers.yaml"
    )

    with open(controller_yaml_path, 'r') as f:
        controller_params = yaml.safe_load(f)

    # Inject use_sim_time into the controller config
    controller_params['use_sim_time'] = use_sim_time

    # Node: robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # # Node: ros2_control_node
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     name='ros2_control_node',
    #     output='screen',
    #     parameters=[
    #         robot_description,
    #         controller_params
    #     ]
    # )

    return [robot_state_publisher]


def generate_launch_description():
    # Paths
    default_urdf_path = PathJoinSubstitution([
        FindPackageShare('mr_pinchy_robot_description'),
        'urdf',
        'my_first_robot.urdf.xacro'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'urdf',
            default_value=default_urdf_path,
            description='Path to URDF/XACRO file'
        ),
        OpaqueFunction(function=launch_setup)
    ])

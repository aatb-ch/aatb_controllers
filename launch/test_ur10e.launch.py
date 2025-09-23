import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('aatb_controllers')
    ur_description_pkg = FindPackageShare('ur_description')

    # Generate robot description using xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([ur_description_pkg, 'urdf', 'ur.urdf.xacro']),
            ' ',
            'ur_type:=ur10e',
            ' ',
            'name:=ur10e',
            ' ',
            'use_fake_hardware:=true',
            ' ',
            'fake_sensor_commands:=false',
            ' ',
            'headless_mode:=true',
            ' ',
            'robot_ip:=0.0.0.0',  # Not used with fake hardware
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'test_ur10e.yaml'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Controller manager with robot description and controller config
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Constrained position controller spawner
    constrained_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['constrained_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay start of controller spawners to ensure controller manager is ready
    delayed_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[constrained_controller_spawner],
                ),
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        delayed_joint_state_spawner,
        delayed_controller_spawner,
    ])
"""Bring up the hexapod in Gazebo Harmonic with ros2_control.

  gazebo (hexapod_world.sdf, has gz-sim-imu-system)
    -> robot_state_publisher (URDF via xacro)
    -> create (spawn from /robot_description)
    -> joint_state_broadcaster -> hexapod_position_controller
  plus a ros_gz_bridge for /clock and /hexapod/imu.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_description')

    # The URDF contains $(find hexapod_description)/config/controllers.yaml,
    # so it MUST go through xacro rather than being read as plain text.
    urdf_path = os.path.join(pkg_share, 'urdf', 'hexapod.urdf')
    robot_description = xacro.process_file(urdf_path).toxml()

    world_path = os.path.join(pkg_share, 'worlds', 'hexapod_world.sdf')

    # Let Gazebo resolve package://hexapod_description/CAD/... mesh URIs.
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(
            [os.path.dirname(pkg_share)]
            + ([os.environ['GZ_SIM_RESOURCE_PATH']]
               if 'GZ_SIM_RESOURCE_PATH' in os.environ else [])
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': [
                LaunchConfiguration('gz_args'), ' ',
                world_path, ' --physics-engine gz-physics-dartsim-plugin',
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-z', '0.15',
        ],
        output='screen',
    )

    # /clock keeps every use_sim_time node ticking ("No clock received").
    # /hexapod/imu comes from the <sensor type="imu"> on imu_link.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/hexapod/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    hexapod_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_position_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r',
            description='Extra gz sim args; use "-r -s --headless-rendering" for headless.'),
        resource_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        # controller_manager only exists once the model is in the world
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[hexapod_position_controller])),
    ])

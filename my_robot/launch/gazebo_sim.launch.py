#!/usr/bin/env python3
"""
Launch Gazebo simulation with the robot arm, ArUco marker world,
camera bridge, and bidirectional joint control.

Flow:
  joint_state_publisher_gui ──→ /joint_states
      │                              │
      │                    ┌─────────┴──────────┐
      │                    │  joint_command_relay │
      │                    └─────────┬──────────┘
      │                              │ /<joint>/cmd_pos (ROS)
      │                    ┌─────────┴──────────┐
      │                    │   ros_gz_bridge     │
      │                    └─────────┬──────────┘
      │                              │ cmd_pos (GZ)
      │                    ┌─────────┴──────────┐
      │                    │     Gazebo arm      │
      │                    │  JointPositionCtrl  │
      │                    └─────────────────────┘
      │
      └──────────────→ robot_state_publisher ──→ TF
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_my_robot, 'worlds', 'aruco_world.sdf')
    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'my.urdf')
    models_path = os.path.join(pkg_my_robot, 'models')

    # Set GZ_SIM_RESOURCE_PATH to include our models directory
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )

    # Help Gazebo find our custom DirectJointPositionPlugin
    pkg_prefix = os.path.normpath(os.path.join(pkg_my_robot, '..', '..'))
    plugin_lib = os.path.join(pkg_prefix, 'lib', 'direct_joint_position_plugin')
    plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib
    )
    ld_library = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=f'{plugin_lib}:{os.environ.get("LD_LIBRARY_PATH", "")}'
    )

    # Fix libEGL warnings on NVIDIA: point glvnd directly to the NVIDIA EGL driver
    egl_vendor = SetEnvironmentVariable(
        name='__EGL_VENDOR_LIBRARY_FILENAMES',
        value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    )

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Enable joint_state_publisher_gui'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file,
        description='Path to the SDF world file'
    )

    # Start Gazebo with the world (robot arm is included in the world)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r --render-engine ogre2 ', world_file],
        }.items(),
    )

    # Load URDF once — needed by robot_state_publisher and joint_state_publisher_gui
    robot_description = open(urdf_file).read()

    # Robot state publisher - reads /joint_states to publish TF frames
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Joint state publisher GUI - for manual joint control
    # Needs robot_description to know joint names and limits from the URDF
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Joint command relay: /joint_states → /<joint>/cmd_pos
    joint_relay = Node(
        package='my_robot',
        executable='joint_command_relay.py',
        name='joint_command_relay',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Bridge: camera topics + joint commands
    # Uses a YAML config so we can map different ROS and Gazebo topic names.
    # JointPositionController listens on /model/<model>/joint/<joint>/0/cmd_pos
    # (token "0" is invalid in a ROS 2 topic name), so the relay publishes to
    # clean /joint/<name>/cmd_pos topics and the bridge remaps them to Gazebo paths.
    bridge_config = os.path.join(pkg_my_robot, 'config', 'gazebo_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_my_robot, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ArUco marker detection node
    aruco_detector = Node(
        package='my_robot',
        executable='aruco_detector.py',
        name='aruco_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Static TF: ArUco marker pose in world frame
    # (from aruco_world.sdf: pose 0.4 0.0 0.25 0 0 0)
    marker_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_world_tf',
        arguments=[
            '0.4', '0.0', '0.25', '0', '0', '0',
            'world', 'aruco_marker_world',
        ],
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    ld.add_action(resource_path)
    ld.add_action(plugin_path)
    ld.add_action(ld_library)
    ld.add_action(egl_vendor)
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(gui_arg)
    ld.add_action(gz_sim)
    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_gui)
    ld.add_action(joint_relay)
    ld.add_action(marker_tf)
    # Start bridge after Gazebo is up
    ld.add_action(TimerAction(period=3.0, actions=[bridge]))
    # Start ArUco detector after bridge is up
    ld.add_action(TimerAction(period=5.0, actions=[aruco_detector]))
    ld.add_action(rviz)

    return ld

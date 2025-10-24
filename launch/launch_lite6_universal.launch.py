#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch: Universal Lite6 Bringup (ROS 2 Jazzy)
- Supports both fake and real hardware
- Uses robot_ip parameter to determine hardware type
- Handles mesh synchronization
- Sets up inertial configuration symlinks
- Configures URDF with proper path handling
- Starts ros2_control_node, robot_state_publisher, RViz2
- Spawns controllers in proper sequence
- Professional error handling and logging
"""

import os
import shutil
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction, 
    TimerAction, 
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration, 
    Command, 
    PathJoinSubstitution, 
    TextSubstitution, 
    FindExecutable,
    PythonExpression
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _setup_prerequisites(context, *args, **kwargs):
    """
    Professional setup function that handles:
    1. Mesh synchronization from xarm_ros2 to installed xarm_description
    2. Inertial configuration symlink setup
    3. Path validation and error handling
    """
    try:
        # Package paths
        my_share = Path(get_package_share_directory("lite6xlerobi"))
        xarm_share = Path(get_package_share_directory("xarm_description"))
        
        # Source and destination paths
        home = Path.home()
        src_meshes = home / "ws/src/xarm_ros2/xarm_description/meshes"
        dst_meshes = xarm_share / "meshes"
        
        # Inertial configuration setup
        inertial_src = my_share / "config/link_inertial/lite6_default_inertial.yaml"
        inertial_dir = xarm_share / "config/link_inertial"
        inertial_dst = inertial_dir / "lite6_default_inertial.yaml"
        
        print("[SETUP] Synchronizing meshes from xarm_ros2...")
        if src_meshes.exists():
            if dst_meshes.exists():
                shutil.rmtree(dst_meshes)
            shutil.copytree(src_meshes, dst_meshes)
            print(f"  ✓ Meshes synchronized: {dst_meshes}")
        else:
            print(f"  ⚠ Source meshes not found: {src_meshes}")
            
        print("[SETUP] Configuring inertial parameters...")
        inertial_dir.mkdir(parents=True, exist_ok=True)
        
        if inertial_dst.exists() or inertial_dst.is_symlink():
            inertial_dst.unlink()
            
        if inertial_src.exists():
            inertial_dst.symlink_to(inertial_src)
            print(f"  ✓ Inertial symlink created: {inertial_dst} -> {inertial_src}")
        else:
            print(f"  ⚠ Inertial source not found: {inertial_src}")
            
        print("[SETUP] Prerequisites completed successfully")
        
    except Exception as e:
        print(f"[ERROR] Setup failed: {e}")
        raise
    
    return []


def _is_fake_hardware(context, *args, **kwargs):
    """Check if we should use fake hardware based on robot_ip parameter"""
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    return robot_ip == "" or robot_ip.lower() == "none" or robot_ip.lower() == "fake"


def generate_launch_description():
    """
    Generate a comprehensive launch description for the Lite6 robot.
    Supports both fake and real hardware based on robot_ip parameter.
    """
    pkg_name = "lite6xlerobi"
    
    # Launch arguments with descriptions
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="",
        description="IP address of the real robot. If empty/none/fake, uses fake hardware"
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2 for visualization"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory(pkg_name),
            "rviz", 
            "standard.rviz"
        ]),
        description="Path to RViz configuration file"
    )
    
    with_gripper_arg = DeclareLaunchArgument(
        "with_gripper",
        default_value="false",
        description="Enable gripper attachment and controller"
    )
    
    with_vacuum_arg = DeclareLaunchArgument(
        "with_vacuum",
        default_value="false", 
        description="Enable vacuum gripper attachment and controller"
    )
    
    robot_dof_arg = DeclareLaunchArgument(
        "robot_dof",
        default_value="6",
        description="Robot degrees of freedom (6 for Lite6)"
    )
    
    # Launch configurations
    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    with_gripper = LaunchConfiguration("with_gripper")
    with_vacuum = LaunchConfiguration("with_vacuum")
    robot_dof = LaunchConfiguration("robot_dof")
    
    # Create conditions for hardware selection
    use_fake_hardware = PythonExpression([
        "'", robot_ip, "'", " == '' or '", robot_ip, "'", ".lower() == 'fake' or '", robot_ip, "'", ".lower() == 'none'"
    ])
    
    use_real_hardware = PythonExpression([
        "'", robot_ip, "'", " != '' and '", robot_ip, "'", ".lower() != 'fake' and '", robot_ip, "'", ".lower() != 'none'"
    ])
    
    # Package paths
    share_dir = get_package_share_directory(pkg_name)
    xarm_share = get_package_share_directory("xarm_description")
    
    # URDF generation for FAKE hardware
    fake_xacro_file = PathJoinSubstitution([
        share_dir, 
        "urdf", 
        "lite6_fake", 
        "lite6_fake_wrapper.urdf.xacro"
    ])
    
    # URDF generation for REAL hardware
    real_xacro_file = PathJoinSubstitution([
        share_dir, 
        "urdf", 
        "lite6_real", 
        "lite6_real_wrapper.urdf.xacro"
    ])
    
    kinematics_file = PathJoinSubstitution([
        xarm_share,
        "config",
        "kinematics", 
        "default",
        "lite6_default_kinematics.yaml"
    ])
    
    # FAKE robot description
    fake_robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            TextSubstitution(text=" "),
            fake_xacro_file,
            TextSubstitution(text=" use_fake_hardware:=true"),
            TextSubstitution(text=" prefix:="),
            TextSubstitution(text=" mesh_path:=package://xarm_description/meshes"),
            TextSubstitution(text=" KIN_YAML:="), kinematics_file,
            TextSubstitution(text=" INERTIAL_YAML:=lite6_default_inertial"),
            TextSubstitution(text=" with_gripper:="), with_gripper,
            TextSubstitution(text=" with_vacuum:="), with_vacuum,
        ]),
        value_type=str,
    )
    
    # REAL robot description
    real_robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            TextSubstitution(text=" "),
            real_xacro_file,
            TextSubstitution(text=" robot_ip:="), robot_ip,
            TextSubstitution(text=" report_type:=dev"),
            TextSubstitution(text=" dof:="), robot_dof,
            TextSubstitution(text=" prefix:="),
            TextSubstitution(text=" mesh_path:=package://xarm_description/meshes"),
            TextSubstitution(text=" KIN_YAML:="), kinematics_file,
            TextSubstitution(text=" INERTIAL_YAML:=lite6_default_inertial"),
            TextSubstitution(text=" with_gripper:="), with_gripper,
            TextSubstitution(text=" with_vacuum:="), with_vacuum,
        ]),
        value_type=str,
    )
    
    # Setup prerequisites
    setup_action = OpaqueFunction(function=_setup_prerequisites)
    
    # Controller configurations
    fake_controller_config = PathJoinSubstitution([
        share_dir,
        "config", 
        "lite6_controllers.yaml"
    ])
    
    real_controller_config = PathJoinSubstitution([
        share_dir,
        "config", 
        "lite6_real_controllers.yaml"
    ])
    
    # FAKE Hardware ros2_control node
    fake_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": fake_robot_description},
            fake_controller_config
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(use_fake_hardware)
    )
    
    # REAL Hardware ros2_control node  
    real_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": real_robot_description},
            real_controller_config
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(use_real_hardware)
    )
    
    # Robot state publisher - works for both fake and real
    # We need separate RSP nodes for fake and real hardware
    fake_rsp_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": fake_robot_description,
            "use_sim_time": False,
            "publish_frequency": 50.0
        }],
        output="screen",
        respawn=False,
        condition=IfCondition(use_fake_hardware)
    )
    
    real_rsp_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": real_robot_description,
            "use_sim_time": False,
            "publish_frequency": 50.0
        }],
        output="screen",
        respawn=False,
        condition=IfCondition(use_real_hardware)
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2", 
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
        respawn=False,
    )
    
    # Controller spawners with proper sequencing
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        output="screen",
    )
    
    arm_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager", 
            "--controller-manager-timeout", "30"
        ],
        output="screen",
    )
    
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=[
            "authentic_lite6_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        condition=IfCondition(with_gripper),
        output="screen",
    )
    
    vacuum_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "vacuum_controller", 
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"
        ],
        condition=IfCondition(with_vacuum),
        output="screen",
    )
    
    # Graceful shutdown when RViz closes
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    
    # Launch description assembly
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(robot_ip_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(rviz_config_arg) 
    ld.add_action(with_gripper_arg)
    ld.add_action(with_vacuum_arg)
    ld.add_action(robot_dof_arg)
    
    # Add setup and core nodes
    ld.add_action(setup_action)
    ld.add_action(fake_control_node)
    ld.add_action(real_control_node)
    
    # Staggered startup for stability
    ld.add_action(TimerAction(
        period=1.0, 
        actions=[fake_rsp_node, real_rsp_node]
    ))
    
    ld.add_action(TimerAction(
        period=1.5, 
        actions=[rviz_node]
    ))
    
    ld.add_action(TimerAction(
        period=2.0, 
        actions=[jsb_spawner]
    ))
    
    ld.add_action(TimerAction(
        period=3.0, 
        actions=[arm_spawner, gripper_spawner, vacuum_spawner]
    ))
    
    # Add event handlers
    ld.add_action(shutdown_on_rviz_exit)
    
    return ld
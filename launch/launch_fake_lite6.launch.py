#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch: Fake-Lite6 Bringup (ROS 2 Jazzy)
- Synchronisiert xarm_description/meshes
- Setzt Symlink für Inertial-Konfiguration
- Führt bringup_prepare.sh aus (erzeugt /tmp/lite6_fake_cfg/*)
- Startet ros2_control_node, robot_state_publisher, RViz2
- Spawnt joint_state_broadcaster und arm_controller nacheinander
"""

import os
import shutil
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)


def _prepare_files_and_params(context, *args, **kwargs):
    """
    Entspricht den Schritten 2–4 im Shell-Script:
      - Meshes syncen
      - Inertial-Link setzen
      - bringup_prepare.sh ausführen
    """
    # --- [Pfad-Auflösung] ---
    home = Path.home()

    # Quelle der Meshes (wie im Bash-Script)
    src_meshes = home / "ws/src/xarm_ros2/xarm_description/meshes"

    # Zielpfade in der installierten xarm_description
    xarm_share = Path(get_package_share_directory("xarm_description"))
    dst_meshes = xarm_share / "meshes"

    # Inertial-Link: Quelle im eigenen Paket, Ziel in xarm_description
    my_share = Path(get_package_share_directory("lite6xlerobi"))
    inertial_src = my_share / "config/link_inertial/auto_lite6_inertial.yaml"
    inertial_dir = xarm_share / "config/link_inertial"
    inertial_dst = inertial_dir / "lite6_default_inertial.yaml"

    print("[2/6] Meshes synchronisieren (xarm_description)...")
    if not src_meshes.exists():
        print(f"  ⚠ Quelle nicht gefunden: {src_meshes}")
    else:
        # rsync --delete Äquivalent: Zielordner komplett neu kopieren
        if dst_meshes.exists():
            shutil.rmtree(dst_meshes)
        shutil.copytree(src_meshes, dst_meshes)
        print(f"  ✓ Meshes gespiegelt nach: {dst_meshes}")

    print("[3/6] Inertial-Link prüfen/setzen...")
    inertial_dir.mkdir(parents=True, exist_ok=True)
    try:
        if inertial_dst.exists() or inertial_dst.is_symlink():
            inertial_dst.unlink()
        inertial_dst.symlink_to(inertial_src)
        print(f"  ✓ Symlink gesetzt: {inertial_dst} -> {inertial_src}")
    except Exception as e:
        print(f"  ⚠ Konnte Symlink nicht setzen: {e}")

    print("[4/6] URDF + Parameter vorbereiten (bringup_prepare.sh)...")
    try:
        subprocess.run(
            ["ros2", "run", "lite6xlerobi", "bringup_prepare.sh"],
            check=True,
        )
        print("  ✓ bringup_prepare.sh abgeschlossen")
    except subprocess.CalledProcessError as e:
        print(f"  ❌ bringup_prepare.sh fehlgeschlagen: {e}")

    return []  # keine Launch-Actions aus dieser Funktion zurückgeben


def generate_launch_description() -> LaunchDescription:
    # Pfade
    pkg_share = Path(get_package_share_directory("lite6xlerobi"))
    rviz_cfg = pkg_share / "rviz/standard.rviz"  # bevorzugt aus dem installierten Paket
    if not rviz_cfg.exists():
        # Fallback: Entwicklungs-Pfad wie in deinem Beispiel
        dev_rviz = Path.home() / "ws/src/lite6xlerobi/rviz/standard.rviz"
        if dev_rviz.exists():
            rviz_cfg = dev_rviz

    # Dateien, die bringup_prepare.sh erzeugt
    params_with_urdf = "/tmp/lite6_fake_cfg/params_with_urdf.yaml"
    rsp_params = "/tmp/lite6_fake_cfg/rsp.yaml"

    # [1/6] Vorbereitung & Files (OpaqueFunction, läuft vor den Nodes)
    prep_step = OpaqueFunction(function=_prepare_files_and_params)

    # [5/6] Controller Manager (ros2_control_node)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[params_with_urdf],
        # name="ros2_control_node"  # optional
    )

    # [6/6] Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[rsp_params],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_cfg)],
    )

    # Spawner (als Nodes statt ros2 run ...)
    # Wir warten kurz, bis controller_manager lauscht:
    spawn_jsb = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout",
                    "30",
                ],
            )
        ],
    )

    # Den Arm-Controller erst nach JSB spawnen
    spawn_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb.actions[0],
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        "arm_controller",
                        "--controller-manager-timeout",
                        "30",
                    ],
                )
            ],
        )
    )

    # Sauber herunterfahren, wenn rviz geschlossen wird
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    ld = LaunchDescription()
    ld.add_action(prep_step)
    ld.add_action(control_node)
    ld.add_action(TimerAction(period=1.0, actions=[rsp_node]))  # entspricht deinem sleep 1
    ld.add_action(TimerAction(period=1.0, actions=[rviz_node]))
    ld.add_action(spawn_jsb)
    ld.add_action(spawn_arm)
    ld.add_action(shutdown_on_rviz_exit)
    return ld

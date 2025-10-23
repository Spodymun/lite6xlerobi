#!/usr/bin/env bash
set -euo pipefail

echo "[1/6] Umgebung laden..."
set +u
source /opt/ros/jazzy/setup.bash
[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash
set -u

echo "[2/6] Meshes synchronisieren (xarm_description)..."
XDESC_SHARE="$(ros2 pkg prefix xarm_description)/share/xarm_description"
rsync -av --delete ~/ws/src/xarm_ros2/xarm_description/meshes/ \
  "$XDESC_SHARE/meshes/" >/dev/null

echo "[3/6] Inertial-Link prüfen/setzen..."
MY_SHARE="$(ros2 pkg prefix lite6xlerobi)/share/lite6xlerobi"
mkdir -p "$XDESC_SHARE/config/link_inertial"
ln -sf "$MY_SHARE/config/link_inertial/auto_lite6_inertial.yaml" \
       "$XDESC_SHARE/config/link_inertial/lite6_default_inertial.yaml"

echo "[4/6] URDF + Parameter vorbereiten..."
ros2 run lite6xlerobi bringup_prepare.sh

echo "[5/6] Controller Manager starten..."
# Starte ros2_control_node in eigenem Terminal
gnome-terminal -- bash -c "
  set +u; source /opt/ros/jazzy/setup.bash; source ~/ws/install/setup.bash; set -u;
  echo '[ros2_control_node] gestartet...';
  ros2 run controller_manager ros2_control_node --ros-args --params-file /tmp/lite6_fake_cfg/params_with_urdf.yaml
" &

sleep 3

echo "[6/6] Robot State Publisher + RViz starten..."

# Robot State Publisher – mit rsp.yaml (robust)
gnome-terminal -- bash -lc '
  set +u; source /opt/ros/jazzy/setup.bash; source ~/ws/install/setup.bash; set -u;
  echo "[robot_state_publisher] startet mit /tmp/lite6_fake_cfg/rsp.yaml";
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file /tmp/lite6_fake_cfg/rsp.yaml
  exec bash
' &

sleep 1

# RViz separat
gnome-terminal -- bash -lc '
  set +u; source /opt/ros/jazzy/setup.bash; source ~/ws/install/setup.bash; set -u;
  echo "[RViz] öffnet...";
  rviz2
  exec bash
' &

# Controller-Spawner separat (mit Timeout & sequentiell)
gnome-terminal -- bash -lc '
  set +u; source /opt/ros/jazzy/setup.bash; source ~/ws/install/setup.bash; set -u;
  echo "[Controller-Spawner] wartet auf controller_manager und startet Controller...";
  # Etwas Luft geben, bis ros2_control_node läuft:
  sleep 2
  ros2 run controller_manager spawner joint_state_broadcaster --controller-manager-timeout 30 && \
  ros2 run controller_manager spawner arm_controller --controller-manager-timeout 30
  exec bash
' &

echo "✅ Setup abgeschlossen!"


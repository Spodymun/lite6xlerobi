#!/usr/bin/env bash
set -euo pipefail

# --- ENV ---
set +u
source /opt/ros/jazzy/setup.bash
[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash
set -u

# --- Pfade ---
MY_SHARE="$(ros2 pkg prefix lite6xlerobi)/share/lite6xlerobi"
XDESC_SHARE="$(ros2 pkg prefix xarm_description)/share/xarm_description"

URDF_SRC="$MY_SHARE/urdf/lite6_fake/lite6_fake_wrapper.urdf.xacro"
KIN_YAML="$XDESC_SHARE/config/kinematics/default/lite6_default_kinematics.yaml"
INERT_KEY="lite6_default_inertial"   # key, kein .yaml !
INERT_LINK_TARGET="$MY_SHARE/config/link_inertial/${INERT_KEY}.yaml"

CFG_DIR="/tmp/lite6_fake_cfg"
URDF="/tmp/lite6_fake_hw.urdf"
COMBO="$CFG_DIR/params_with_urdf.yaml"
RSP="$CFG_DIR/rsp.yaml"
mkdir -p "$CFG_DIR"

rsync -av "$HOME/ws/src/xarm_ros2/xarm_description/meshes/" "$XDESC_SHARE/meshes/" >/dev/null || true

# --- 1) Inertial-Link setzen (Makros erwarten link_inertial/<KEY>.yaml) ---
mkdir -p "$XDESC_SHARE/config/link_inertial"
ln -sf "$INERT_LINK_TARGET" "$XDESC_SHARE/config/link_inertial/${INERT_KEY}.yaml"
echo "[OK] INERTIAL link: $XDESC_SHARE/config/link_inertial/${INERT_KEY}.yaml -> $INERT_LINK_TARGET"

# --- 2) URDF rendern ---
xacro "$URDF_SRC" \
  use_fake_hardware:=true \
  prefix:="" \
  mesh_path:=package://xarm_description/meshes \
  KIN_YAML:="$KIN_YAML" \
  INERTIAL_YAML:="$INERT_KEY" \
  > "$URDF"

# --- 2b) absolute → package:// (immer erzwingen) ---
sed -i 's#filename="/home/[^"]*/share/xarm_description/meshes/#filename="package://xarm_description/meshes/#g' "$URDF"

grep -m3 -n 'package://xarm_description/meshes' "$URDF"
echo "[1/3] URDF rendern"

# --- 3) ros2_control-Params mit URDF einbetten ---
cat > "$COMBO" <<'YAML'
controller_manager:
  ros__parameters:
    update_rate: 100
    robot_description: |
YAML
sed 's/^/      /' "$URDF" >> "$COMBO"
cat >> "$COMBO" <<'YAML'

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_partial_joints_goal: false
    interpolate_from_desired_state: true
YAML
echo "[2/3] ros2_control Params mit URDF einbetten"

# --- 4) robot_state_publisher-Params (NEU/FEHLTE) ---
{
  echo 'robot_state_publisher:'
  echo '  ros__parameters:'
  echo '    robot_description: |'
  sed 's/^/      /' "$URDF"
} > "$RSP"

echo "[3/3] Artefakte:"
ls -lh "$COMBO" "$URDF" "$RSP"


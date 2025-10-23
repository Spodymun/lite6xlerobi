#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash"
set -u

MY_SHARE="$(ros2 pkg prefix lite6xlerobi)/share/lite6xlerobi"
XDESC_SHARE="$(ros2 pkg prefix xarm_description)/share/xarm_description"

SRC="$MY_SHARE/config/link_inertial/lite6_default_inertial.yaml"
DEST_DIR="$XDESC_SHARE/config/link_inertial"
DEST="$DEST_DIR/lite6_default_inertial.yaml"

mkdir -p "$DEST_DIR"
ln -sf "$SRC" "$DEST"
echo "[OK] INERTIAL link: $DEST -> $SRC"

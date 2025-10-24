# Raspberry Pi Setup für Lite6 xArm Roboter

Komplette Schritt-für-Schritt Anleitung zur Einrichtung eines frischen Raspberry Pi mit Ubuntu 24.04 für die Lite6 xArm Robotersteuerung.

## 🚀 Step-by-Step Installation

### 1. System aktualisieren

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

Nach dem Neustart wieder einloggen und fortfahren.

### 2. Grundlegende Tools installieren

```bash
# Entwicklungstools und Dependencies
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    nano \
    vim \
    htop \
    net-tools
```

### 3. ROS2 Jazzy Installation (Offizielle Methode)

#### 3.1 Locale Setup
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

#### 3.2 Repositories aktivieren
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

#### 3.3 ROS2 Jazzy Installation
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

#### 3.4 Zusätzliche ROS2 Pakete für xArm
```bash
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-position-controllers \
    ros-jazzy-effort-controllers \
    ros-jazzy-gripper-controllers \
    python3-colcon-common-extensions
```

### 4. ROS2 Environment Setup

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Workspace erstellen

```bash
mkdir -p ~/ws/src
cd ~/ws
```

### 6. xArm ROS2 Repository klonen

```bash
cd ~/ws/src
git clone https://github.com/xArm-Developer/xarm_ros2.git
cd xarm_ros2
git checkout jazzy
cd ..
```

### 7. Lite6xlerobi Repository klonen

```bash
cd ~/ws/src
git clone https://github.com/Spodymun/lite6xlerobi.git
```

### 8. Dependencies installieren

```bash
cd ~/ws

rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

pip3 install -U \
    setuptools \
    wheel \
    numpy \
    scipy \
    pyyaml
```

### 9. Workspace bauen

```bash
cd ~/ws
colcon build --symlink-install
```

### 10. Environment Setup für Workspace

```bash
echo "source ~/ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 11. Netzwerk-Konfiguration (für echte Hardware)

```bash
# IP-Konfiguration prüfen
ip addr show

# Für statische IP (optional, je nach Netzwerk-Setup):
# sudo nano /etc/netplan/01-netcfg.yaml
# Dann entsprechend konfigurieren
```

### 12. Installation testen

```bash
# Test 1: ROS2 Funktionalität
ros2 topic list

# Test 2: Fake Hardware
ros2 launch lite6xlerobi launch_lite6_universal.launch.py

# Test 3.1: Mit Gripper
ros2 launch lite6xlerobi launch_lite6_universal.launch.py with_gripper:=true

# Test 3.1: Mit Gripper
ros2 launch lite6xlerobi launch_lite6_universal.launch.py with_vacuum:=true

# Test 4: Controller-Status prüfen
ros2 control list_controllers

# Test 5: Joint States prüfen
ros2 topic echo /joint_states

```

## 🔧 Troubleshooting

### Problem: USB/Serial Permissions (Real Robot)

```bash
# User zur dialout Gruppe hinzufügen
sudo usermod -a -G dialout $USER

# Logout/Login erforderlich
```
---

## 🎯 Quick Start Commands

Nach erfolgreicher Installation:

### Fake Hardware (Simulation)
```bash
ros2 launch lite6xlerobi launch_lite6_universal.launch.py with_gripper:=true
```

### Echte Hardware
```bash
ros2 launch lite6xlerobi launch_lite6_universal.launch.py robot_ip:=192.168.1.186
```

### Mit Gripper
```bash
ros2 launch lite6xlerobi launch_lite6_universal.launch.py robot_ip:=192.168.1.186 with_gripper:=true
```

---

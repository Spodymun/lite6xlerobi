cmake_minimum_required(VERSION 3.8)
project(lite6xlerobi)

# ROS 2 Buildsystem
find_package(ament_cmake REQUIRED)

# Installiere ausführbare Skripte für ros2 run
install(PROGRAMS
  scripts/bringup_prepare.sh
  scripts/republish_description.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/bootstrap_inertial.sh DESTINATION lib/${PROJECT_NAME})
ament_package()

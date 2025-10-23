cmake_minimum_required(VERSION 3.8)
project(lite6xlerobi)

find_package(ament_cmake REQUIRED)

# Ausführbare Skripte, die mit `ros2 run` aufrufbar sein sollen
install(PROGRAMS
  scripts/bringup_prepare.sh
  scripts/republish_description.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ressourcen ins share/ legen
install(DIRECTORY
  urdf
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


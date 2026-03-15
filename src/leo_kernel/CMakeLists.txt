cmake_minimum_required(VERSION 3.8)
project(leo_kernel)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install executable scripts
install(PROGRAMS
  scripts/mission_manager.py
  scripts/gps_monitor.py
  scripts/tf_mode_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

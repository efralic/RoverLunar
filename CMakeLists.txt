cmake_minimum_required(VERSION 3.8)
project(lunar_rover_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY urdf worlds launch config maps meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mission_manager.py
  scripts/rock_detector.py
  scripts/arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(lunar_rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Instalar directorios de recursos
install(DIRECTORY
  urdf
  worlds
  launch
  config
  scripts
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts Python como ejecutables
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/mission_manager.py
  scripts/rock_detector.py
  scripts/arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

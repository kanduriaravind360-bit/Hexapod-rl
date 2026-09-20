cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Robot description, meshes, bringup, controller config and the Gazebo world.
install(
  DIRECTORY urdf CAD launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Executable helper nodes (ros2 run hexapod_description stand.py)
install(
  PROGRAMS scripts/stand.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

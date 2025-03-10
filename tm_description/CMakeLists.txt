cmake_minimum_required(VERSION 3.10.2)
project(tm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY meshes urdf xacro 
    DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
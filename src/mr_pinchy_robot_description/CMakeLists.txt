cmake_minimum_required(VERSION 3.8)
project(mr_pinchy_robot_description)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# # Install URDF files
# install(DIRECTORY urdf/
#   DESTINATION share/${PROJECT_NAME}/urdf
# )

# # Install mesh files
# install(DIRECTORY meshes/
#   DESTINATION share/${PROJECT_NAME}/meshes
# )

# # Install launch files
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Install RViz config files
# install(DIRECTORY rviz/
#   DESTINATION share/${PROJECT_NAME}/rviz
# )
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )
install(
  DIRECTORY launch config meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)


# Package declaration
ament_package()

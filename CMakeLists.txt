cmake_minimum_required(VERSION 3.8)
project(final_project_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  config
  meshes
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/activate_nav.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

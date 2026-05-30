cmake_minimum_required(VERSION 3.8)
project(my_ugv_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories to the share folder
install(DIRECTORY 
  launch 
  config 
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_nav_relay.py
  scripts/joy_speed_estop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

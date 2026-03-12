cmake_minimum_required(VERSION 3.8)
project(my_ugv_description)

# 1. Find the ROS 2 build tools
find_package(ament_cmake REQUIRED)

# 2. INSTALL THE URDF FOLDER
# This moves your 'urdf' folder from /src to /install
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 3. INSTALL THE PACKAGE.XML
# ROS 2 needs this file in the install folder to recognize the package
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4. Finalize the package
ament_package()


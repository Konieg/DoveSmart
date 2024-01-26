cmake_minimum_required(VERSION 3.5)
project(origincar_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(actionlib REQUIRED)
#find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)                         # CHANGE
find_package(origincar_msg REQUIRED)                         # CHANGE
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(origincar_description REQUIRED)
find_package(robot_localization REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
 )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)
set(origincar_base_node_SRCS
  src/origincar_base.cpp
  src/Quaternion_Solution.cpp
)

add_executable(origincar_base_node src/origincar_base.cpp src/Quaternion_Solution.cpp)
ament_target_dependencies(origincar_base_node tf2_ros tf2 tf2_geometry_msgs rclcpp std_msgs robot_localization nav_msgs std_srvs sensor_msgs ackermann_msgs serial origincar_msg origincar_description)

#add_executable(testNode src/test.cpp src/Quaternion_Solution.cpp)
#ament_target_dependencies(testNode rclcpp std_msgs nav_msgs std_srvs sensor_msgs ackermann_msgs serial origincar_msg)

install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS
origincar_base_node

DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch  config 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

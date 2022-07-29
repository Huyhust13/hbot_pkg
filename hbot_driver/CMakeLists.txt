cmake_minimum_required(VERSION 3.5)
project(hbot_driver)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hbot_msg REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hbot_driver src/hbot_driver_node.cpp)
ament_target_dependencies(hbot_driver rclcpp serial hbot_msg)

install(TARGETS
  hbot_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

cmake_minimum_required(VERSION 3.5)
project(limo_control)

# MODIFIED - C++ settings:
# set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(limo_node src/limo_node.cpp)
ament_target_dependencies(limo_node rclcpp geometry_msgs nav_msgs)


install(TARGETS
    limo_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
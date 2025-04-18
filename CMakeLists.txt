cmake_minimum_required(VERSION 3.8)
project(beer_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(arduino_driver src/arduino_driver.cpp)
add_executable(joy_to_twist src/joy_to_twist_node.cpp)

ament_target_dependencies(arduino_driver rclcpp geometry_msgs tf2_ros nav_msgs)
ament_target_dependencies(joy_to_twist rclcpp geometry_msgs sensor_msgs)

install(TARGETS 
        arduino_driver
        joy_to_twist
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

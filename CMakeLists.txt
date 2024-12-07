cmake_minimum_required(VERSION 3.8)
project(beer_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(arduino_driver src/arduino_driver.cpp)
ament_target_dependencies(arduino_driver rclcpp geometry_msgs)

install(TARGETS arduino_driver
  DESTINATION lib/${PROJECT_NAME})


ament_package()

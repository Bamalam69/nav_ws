cmake_minimum_required(VERSION 3.8)
project(inertiallabs_ins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(inertiallabs_interfaces REQUIRED)

# Add the custom message include directory
# and low level sdk
include_directories(
  ${inertiallabs_interfaces_INCLUDE_DIRS}
  ../../external/inertiallabs_sdk/
  )

add_executable(inertiallabs_ins
  src/il_ins.cpp
  ../../external/inertiallabs_sdk/ILDriver.cpp
  ../../external/inertiallabs_sdk/UDDParser.cpp
  ../../external/inertiallabs_sdk/platforms/linux/SerialPort.cpp
  ../../external/inertiallabs_sdk/platforms/linux/NetClient.cpp
)
ament_target_dependencies(inertiallabs_ins
  rclcpp
  sensor_msgs
  inertiallabs_interfaces
  nav_msgs
)

install(TARGETS
  inertiallabs_ins
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

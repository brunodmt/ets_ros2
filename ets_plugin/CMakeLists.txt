cmake_minimum_required(VERSION 3.5)

project(ets_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ets_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_library(etsros2 MODULE src/telemetry.cpp src/publisher.cpp)

ament_target_dependencies(etsros2
    "rclcpp"
    "rcutils"
    "rosidl_typesupport_cpp"
    "std_msgs"
    "nav_msgs"
    "ets_msgs")

target_include_directories(etsros2 PRIVATE external/scs_sdk_1_9/include)

install(TARGETS etsros2
    DESTINATION lib/${PROJECT_NAME})

ament_package()

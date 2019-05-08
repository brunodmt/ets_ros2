cmake_minimum_required(VERSION 3.5)

project(ets_cpp_client)

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
find_package(ets_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(ets_cpp_client src/ets_cpp_client.cpp)

ament_target_dependencies(ets_cpp_client
    "rclcpp"
    "rcutils"
    "rosidl_typesupport_cpp"
    "std_msgs"
    "ets_msgs")

install(TARGETS ets_cpp_client
    DESTINATION bin/${PROJECT_NAME})

ament_package()

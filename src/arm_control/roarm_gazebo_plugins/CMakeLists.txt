cmake_minimum_required(VERSION 3.8)
project(roarm_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
# 【修改1】四路电机转速使用 Float64MultiArray
find_package(std_msgs REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(roarm_quad_motor_model SHARED
  src/roarm_quad_motor_model.cpp
)

target_compile_features(roarm_quad_motor_model PUBLIC cxx_std_17)
target_link_libraries(roarm_quad_motor_model
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(roarm_quad_motor_model
  rclcpp
  gazebo_ros
  gazebo_dev
  geometry_msgs
  std_msgs
)

install(
  TARGETS roarm_quad_motor_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

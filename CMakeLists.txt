cmake_minimum_required(VERSION 3.8)
project(gazebo_ros2_actor_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  gazebo_msgs
  gazebo_ros
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros2_actor_command
add_library(gazebo_ros2_actor_command SHARED
  src/gazebo_ros2_actor_command.cpp
)
target_include_directories(gazebo_ros2_actor_command PUBLIC include)
ament_target_dependencies(gazebo_ros2_actor_command ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(gazebo_ros2_actor_command)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros2_actor_command
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
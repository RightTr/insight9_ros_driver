cmake_minimum_required(VERSION 3.0.2)
project(insight_ros_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROS_EDITION "" CACHE STRING "ROS edition: ROS1 or ROS2")
if(ROS_EDITION STREQUAL "ROS1")
  set(USE_ROS1 ON)
elseif(ROS_EDITION STREQUAL "ROS2")
  set(USE_ROS2 ON)
else()
  find_package(catkin QUIET)
  if(catkin_FOUND)
    set(USE_ROS1 ON)
  else()
    find_package(ament_cmake QUIET)
  endif()
  if(NOT USE_ROS1 AND ament_cmake_FOUND)
    set(USE_ROS2 ON)
  endif()
endif()

find_package(Threads REQUIRED)

if(USE_ROS1)
  message(STATUS "Building insight_ros_driver for ROS1")
  add_definitions(-DUSE_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs
    INCLUDE_DIRS include
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_library(insight9_sdk
    src/Insight_9_receive.cpp
    src/UvcExtensionUnit.cpp
  )
  target_link_libraries(insight9_sdk Threads::Threads)

  add_executable(insight9_ros_driver_node src/insight9_ros_driver_node.cpp)
  target_link_libraries(insight9_ros_driver_node
    insight9_sdk
    Threads::Threads
    ${catkin_LIBRARIES}
  )

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  install(DIRECTORY launch config rviz_cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(TARGETS insight9_sdk insight9_ros_driver_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

if(USE_ROS2)
  message(STATUS "Building insight_ros_driver for ROS2")
  add_definitions(-DUSE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  include_directories(
    include
  )

  add_library(insight9_sdk
    src/Insight_9_receive.cpp
    src/UvcExtensionUnit.cpp
  )
  target_link_libraries(insight9_sdk Threads::Threads)

  add_executable(insight9_ros_driver_node src/insight9_ros_driver_node.cpp)
  ament_target_dependencies(insight9_ros_driver_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    builtin_interfaces
  )
  target_link_libraries(insight9_ros_driver_node
    insight9_sdk
    Threads::Threads
  )

  install(DIRECTORY include/
    DESTINATION include
  )
  install(DIRECTORY launch config rviz_cfg
    DESTINATION share/${PROJECT_NAME}
  )
  install(TARGETS insight9_sdk insight9_ros_driver_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
endif()

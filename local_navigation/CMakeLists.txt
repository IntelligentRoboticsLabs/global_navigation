cmake_minimum_required(VERSION 3.8)
project(local_navigation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  tf2_ros
  tf2_geometry_msgs
  grid_map_ros
  grid_map_msgs
  pcl_conversions
  pcl_ros
  nav_msgs
  image_geometry
  OpenCV
)

include_directories(
  include
)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/GridmapUpdaterNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(local_navigation_program
  src/local_navigation_program.cpp
)
ament_target_dependencies(local_navigation_program ${dependencies})
target_link_libraries(local_navigation_program ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS local_navigation_program
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()

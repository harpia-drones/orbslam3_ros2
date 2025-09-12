cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-reorder)
endif()

# ========================================
#     DEPENDENCIES
# ========================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# OBS.: the ORB_SLAM3 include by itself: OpenCV, Sophus, Eigen, DBoW2, Pangolin, g2o. 
# It is not necessary execute find_packages() for those packages.

# ========================================
#     MONOCULAR
# ========================================

add_executable(monocular
  src/monocular.cpp
)

ament_target_dependencies(monocular
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(monocular
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
)

install(TARGETS 
  monocular
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     MONOCULAR DEBUG
# ========================================

add_executable(monocular_debug
  src/monocular_debug.cpp
)

ament_target_dependencies(monocular_debug
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(monocular_debug
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
)

install(TARGETS 
  monocular_debug
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     LAUNCH
# ========================================

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

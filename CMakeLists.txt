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
#     MONO CAM
# ========================================

add_executable(mono_cam
  src/mono_cam.cpp
)

ament_target_dependencies(mono_cam
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(mono_cam
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
)

install(TARGETS 
  mono_cam
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

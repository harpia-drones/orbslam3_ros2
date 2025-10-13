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
find_package(px4_msgs REQUIRED)

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
  px4_msgs
)

target_link_libraries(monocular
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS 
  monocular
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     MONOCULAR INERTIAL
# ========================================

add_executable(monocular_inertial
  src/monocular_inertial.cpp
)

ament_target_dependencies(monocular_inertial
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

target_link_libraries(monocular_inertial
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS 
  monocular_inertial
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     RGBD INERTIAL
# ========================================

add_executable(rgbd_inertial
  src/rgbd_inertial.cpp
)

ament_target_dependencies(rgbd_inertial
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

target_link_libraries(rgbd_inertial
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS 
  rgbd_inertial
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     RGBD INERTIAL V2
# ========================================

add_executable(rgbd_inertial_v2
  src/rgbd_inertial_v2.cpp
)

ament_target_dependencies(rgbd_inertial_v2
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

target_link_libraries(rgbd_inertial_v2
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS 
  rgbd_inertial_v2
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
#     TESTE GEMINI
# ========================================

add_executable(teste_gemini
  src/teste_gemini.cpp
)

ament_target_dependencies(teste_gemini
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
)

target_link_libraries(teste_gemini
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS 
  teste_gemini
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

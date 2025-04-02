cmake_minimum_required(VERSION 3.8)
project(hive_data_collector)

# Trouver ament_cmake + ament_cmake_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Dépendances C++
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

# ------------------------------------------------------------------------------
# 1) Partie Python : Installe vos modules / scripts
# ------------------------------------------------------------------------------
# Installe le dossier "hive_data_collector/hive_data_collector" comme module Python (si vous avez du code Python dedans).
ament_python_install_package(${PROJECT_NAME})

# Installe les scripts Python "exécutables"
install(
  PROGRAMS
    scripts/rosbag_recorder_manager.py
    scripts/rosbag_player_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 2) Partie C++ : Compilation du node lidar_crop_node
# ------------------------------------------------------------------------------
add_executable(lidar_crop_node src/lidar_crop_node.cpp)
ament_target_dependencies(lidar_crop_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(lidar_crop_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_crop_node
  ${PCL_LIBRARIES}
)

install(
  TARGETS lidar_crop_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Final
# ------------------------------------------------------------------------------
ament_package()

cmake_minimum_required(VERSION 3.0.2)
project(lesta)
add_compile_options(-std=c++17)

# LibTorch setup
find_package(Torch REQUIRED PATHS /opt/libtorch)

# [加入 Torch 的 CXX flags,防止因 CXX11 ABI 导致链接错误]
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    height_mapping_core
    roscpp
    tf2_ros
    pcl_conversions
    grid_map_ros
    message_generation
    cv_bridge
    sensor_msgs
    message_filters
)

add_service_files(
  FILES
  save_training_data.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    height_mapping_core
    roscpp
    tf2_ros
    pcl_conversions
    grid_map_ros
    cv_bridge
    sensor_msgs
    message_filters
    message_runtime
)


###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Node: Label generation
add_executable(label_generation_node 
  src/ros/label_generation_node.cpp
  src/core/HeightMapper.cpp
  src/core/GlobalMapper.cpp
  src/core/FeatureExtractor.cpp
  src/core/LabelGenerator.cpp
)
target_link_libraries(label_generation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  height_mapping_core
  ${OpenCV_LIBRARIES}
)
# Node: add dependencies to service messages header files
add_dependencies(label_generation_node height_mapping_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Traversability prediction
add_executable(trav_prediction_node
  src/ros/trav_prediction_node.cpp
  src/core/HeightMapper.cpp
  src/core/FeatureExtractor.cpp
  src/core/TraversabilityNetwork.cpp
  src/core/TraversabilityEstimator.cpp
)
target_link_libraries(trav_prediction_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  height_mapping_core
  ${TORCH_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
# Node: add message dependencies
add_dependencies(trav_prediction_node height_mapping_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Traversability mapping
add_executable(trav_mapping_node
  src/ros/trav_mapping_node.cpp
  src/core/HeightMapper.cpp
  src/core/GlobalMapper.cpp
  src/core/FeatureExtractor.cpp
  src/core/TraversabilityNetwork.cpp
  src/core/TraversabilityEstimator.cpp
  src/core/TraversabilityMapper.cpp
)
target_link_libraries(trav_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  height_mapping_core
  ${TORCH_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(trav_mapping_node height_mapping_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

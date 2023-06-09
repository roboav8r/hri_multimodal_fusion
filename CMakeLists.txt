cmake_minimum_required(VERSION 3.0.2)
project(hri_multimodal_fusion)

add_compile_options(-std=c++17)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    genmsg 
    geometry_msgs 
    message_generation
    cv_bridge 
    image_transport
    rosbag
)

find_package(GTSAM REQUIRED)
if(NOT GTSAM_INCLUDE_DIRS)
  set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
endif()

find_package(GTSAM_UNSTABLE REQUIRED)
if(NOT GTSAM_UNSTABLE_INCLUDE_DIRS)
  set(GTSAM_UNSTABLE_INCLUDE_DIRS ${GTSAM_UNSTABLE_INCLUDE_DIR})
endif()

include_directories(
    ${catkin_INCLUDE_DIRS} 
    include
)

# Declare ROS messages and services
add_message_files(
    DIRECTORY msg 
    FILES DetectionLabel.msg DetectionLabelArray.msg TrackedObject.msg
)

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES LabelBagData.srv
)

## Generate added messages and services
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs 
    hri_multimodal_fusion
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    DEPENDS GTSAM GTSAM_UNSTABLE
)

add_executable(cal_graph src/cal_graph.cpp)
target_include_directories(cal_graph
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(cal_graph gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(cal_graph gtsam gtsam_unstable)

add_executable(inference_graph src/inference_graph.cpp)
target_include_directories(inference_graph
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(inference_graph gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(inference_graph gtsam gtsam_unstable)

add_executable(inference_filter src/inference_filter.cpp)
target_include_directories(inference_filter
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(inference_filter gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(inference_filter gtsam gtsam_unstable)

add_executable(oakd_cal_graph src/oakd_cal_graph.cpp)
target_include_directories(oakd_cal_graph
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(oakd_cal_graph gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(oakd_cal_graph gtsam gtsam_unstable)

add_executable(oakd_inference_graph src/oakd_inference_graph.cpp)
target_include_directories(oakd_inference_graph
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(oakd_inference_graph gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(oakd_inference_graph gtsam gtsam_unstable)

add_executable(oakd_train_node src/oakd_train.cpp)
target_link_libraries(oakd_train_node ${catkin_LIBRARIES})
add_dependencies(oakd_train_node hri_multimodal_fusion_generate_messages_cpp)

add_executable(perception_training_node src/train_node.cpp)
target_link_libraries(perception_training_node ${catkin_LIBRARIES})
add_dependencies(perception_training_node hri_multimodal_fusion_generate_messages_cpp)


add_executable(tracked_object_viz_node src/tracked_object_viz_node.cpp)
target_include_directories(tracked_object_viz_node
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${GTSAM_UNSTABLE_INCLUDE_DIRS}
)
target_link_libraries(tracked_object_viz_node ${catkin_LIBRARIES})


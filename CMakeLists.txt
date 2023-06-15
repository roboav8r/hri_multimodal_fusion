cmake_minimum_required(VERSION 3.0.2)
project(hri_multimodal_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# prevents a keyword clash between Qt and boost
add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)


set(SRC_FILES
  src/hri_humans.cpp
  src/hri_skeletons.cpp
  src/hri_tf.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})


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


target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

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


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

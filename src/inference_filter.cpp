#include <iostream>
#include <fstream>

#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>

#include "ros/ros.h"
#include "depthai_ros_msgs/SpatialDetectionArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "filters.hpp"

// Constants
std::vector<std::string> topics;
std::string yoloTopic = "yolov4_publisher/color/yolov4_Spatial_detections";

int main(int argc, char **argv) {


    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    InferenceFilter filter;

    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
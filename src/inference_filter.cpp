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

#include "transition_models.hpp"
#include "filters.hpp"
#include "io.hpp"

// Constants
std::string yoloTopic;// = "yolov4_publisher/color/yolov4_Spatial_detections";

//XmlRpc::XmlRpcValue transitionParams;
std::vector<double> motionVar;
std::vector<double> sensorVar;

int main(int argc, char **argv) {

    // Start node
    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    // Generate state transition models from parameter file
    nh.getParam("/transition/sigma", motionVar);
    static TransitionModels::ConstVelMotion cvMotion(motionVar);

    // Generate observation models from parameter file
    nh.getParam("/sensors/topic", yoloTopic);
    nh.getParam("/sensors/sigma",sensorVar);
    static Sensors::OakDSensor oakdModel(sensorVar);

    // Run the filter
    InferenceFilter filter(cvMotion, oakdModel);
    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
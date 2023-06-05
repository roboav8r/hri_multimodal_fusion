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

// Constants
Filters::FilterParams filterParams;
std::vector<double> motionVar;
std::string yoloTopic;
std::vector<double> sensorVar;

int main(int argc, char **argv) {

    // Start node
    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    // Load filter parameters
    filterParams = Filters::ExtractParams("filter/", nh);

    // Generate state transition models from parameter file
    // TODO get length of transition/*/, create model of */type with sigma */sigmas and label *
    nh.getParam("/transition/walking/sigma", motionVar);
    static TransitionModels::ConstVel cvMotion(motionVar);
    // TODO create static vector of transition models? pointers?

    // Generate observation models from parameter file
    nh.getParam("/sensors/topic", yoloTopic);
    nh.getParam("/sensors/sigma",sensorVar);
    static Sensors::OakDSensor oakdModel(sensorVar);
    static Sensors::Clutter3D oakdClutter = Sensors::ExtractClutterParams("clutter/",nh);

    // Run the filter
    // TODO replace cvMotion with model vector and labels
    Filters::InferenceFilter filter(nh, filterParams, cvMotion, oakdModel, oakdClutter);
    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &Filters::InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
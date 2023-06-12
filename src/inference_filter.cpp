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
static Filters::FilterParams filterParams;
// std::vector<double> motionVar;
static TransitionModels::TransModelParams transModelParams;

std::string yoloTopic;
std::vector<double> sensorVar;

int main(int argc, char **argv) {

    // Start node
    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    // Load filter parameters
    filterParams = Filters::ExtractFilterParams("filter/", nh);

    // Generate state transition models from parameter file
    transModelParams = TransitionModels::ExtractTransModelParams("transition/", nh);
    //TransitionModels::PrintTransModelParams(transModelParams);

    // Generate observation models from parameter file
    nh.getParam("/sensors/topic", yoloTopic);
    nh.getParam("/sensors/sigma",sensorVar);
    static Sensors::OakDSensor oakdModel(sensorVar);
    static Sensors::Clutter3D oakdClutter = Sensors::ExtractClutterParams("clutter/",nh);

    // Run the filter
    Filters::InferenceFilter filter(nh, filterParams, transModelParams, oakdModel, oakdClutter);
    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &Filters::InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
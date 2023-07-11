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
static TransitionModels::TransModelParams transModelParams;
static Sensors::ObsModelParams obsParams;
static Sensors::Clutter3D oakdClutter;

int main(int argc, char **argv) {

    // Start node
    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    // Load filter parameters
    filterParams = Filters::ExtractFilterParams("filter/", nh);

    // Generate state transition models from parameter file
    transModelParams = TransitionModels::ExtractTransModelParams("transition/", nh);

    // Generate observation models from parameter file
    obsParams = Sensors::ExtractSensorParams("sensors/",nh);
    oakdClutter = Sensors::ExtractClutterParams("clutter/",nh);

    // Create the filter
    Filters::InferenceFilter filter(nh, filterParams, transModelParams, obsParams, oakdClutter);

    // Set up subscribers & callbacks
    // TODO - make filter.subscribers_ member and push_back this sub; iterate through sensor params.Topics
    // TODO generate callback functions with pointer to filter.update()
    ros::Subscriber yoloSub = nh.subscribe(obsParams.SensorTopics[0], 1, &Filters::InferenceFilter::OakDCallback, &filter);

    ros::spin();
    return 0;

};
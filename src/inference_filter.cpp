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

void oakdCallback(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& msg)
{
    ROS_INFO("Got detection msg");
        // // Get message data
        // depthai_ros_msgs::SpatialDetectionArray::ConstPtr det = m.instantiate<depthai_ros_msgs::SpatialDetectionArray>();
        // std::cout << *det << std::endl;
        // t = det->header.stamp;
        // std::cout << t << std::endl;

        // // Create symbols for state
        // state_symbol = gtsam::Symbol('x',ii);

        // if (ii==1) // if this is the first node, add a measurement factor
        // { 
        //     graph.emplace_shared<OakDInferenceFactor>(state_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z,oakdPosNoise);
        // } 
        // else // For subsequent nodes
        // {
        //     // Add state transition factor and measurement factor
        //     dt = t - last_t;
        //     gtsam::SharedDiagonal Q = 
        //         gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(135.688255*pow(dt.toSec(),2), 
        //                                                            98.0265414*pow(dt.toSec(),2),
        //                                                            395.476227*pow(dt.toSec(),2),
        //                                                            0.100000196*dt.toSec(), 
        //                                                            0.0999837353*dt.toSec(), 
        //                                                            0.0997463441*dt.toSec()), true);
        //     graph.emplace_shared<ConstVelStateTransition>(last_state_symbol,state_symbol,dt.toSec(),Q);

        //     graph.emplace_shared<OakDInferenceFactor>(state_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z,oakdPosNoise);


    // Predict (t)

    // Update if message not empty

    // Publish state
};


int main(int argc, char **argv) {


    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    InferenceFilter filter;

    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
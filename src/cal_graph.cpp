#include <iostream>
#include <fstream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>


#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "depthai_ros_msgs/SpatialDetectionArray.h"

#include "hri_factors.hpp"

// Constants
std::string package_path = ros::package::getPath("hri_multimodal_fusion");
std::string bag_filepath{"/data/test.bag"}; // TODO get as input arg
std::vector<std::string> topics;


int main() {

    // Bagfile variables and parameters
    ros::Time t, last_t;
    ros::Duration dt;
    int n_meas = 5; // TODO replace n_meas with size of view 
    
    // Graph variables and parameters
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::Symbol last_state_symbol, state_symbol, sensor_noise_symbol, proc_noise_symbol;

    // Sensor noise models (variable to be optimized)
    gtsam::Vector3 oakdPosNoise(0.1,0.1,0.1);
    sensor_noise_symbol = gtsam::Symbol('R',1);

    // Process noise model
    gtsam::Vector6 procNoise(0.1,0.1,0.1,0.1,0.1,0.1);
    proc_noise_symbol = gtsam::Symbol('Q',1);

    // Read bag file
    rosbag::Bag bag;
    bag.open(package_path + bag_filepath, rosbag::bagmode::Read);
    topics.push_back(std::string("yolov4_publisher/color/yolov4_Spatial_detections"));
    rosbag::View view(bag, rosbag::TopicQuery(topics)); // TODO replace this with rosbag::View(bag) to get all topics

    // Iterate through bagfile messages
    for(unsigned short ii=1; rosbag::MessageInstance const m : view)
    {
        // Get message data
        depthai_ros_msgs::SpatialDetectionArray::ConstPtr det = m.instantiate<depthai_ros_msgs::SpatialDetectionArray>();
        std::cout << *det << std::endl;
        t = det->header.stamp;
        std::cout << t << std::endl;

        // Create symbols for state
        state_symbol = gtsam::Symbol('x',ii);

        if (ii==1) // if this is the first node, add a measurement factor
        { 
            graph.emplace_shared<OakDCalibrationFactor>(state_symbol,sensor_noise_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z);
        } 
        else // For subsequent nodes
        {
            // Add state transition factor and measurement factor
            dt = t - last_t;
            //gtsam::SharedDiagonal Q = 
                //gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(0.1*pow(dt.toSec(),2), 0.1*pow(dt.toSec(),2), 0.1*pow(dt.toSec(),2), 0.1*dt.toSec(), 0.1*dt.toSec(), 0.1*dt.toSec()), true);
            graph.emplace_shared<ConstVelStateTransCal>(last_state_symbol,state_symbol,proc_noise_symbol,dt.toSec());

            graph.emplace_shared<OakDCalibrationFactor>(state_symbol,sensor_noise_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z);


        }

        // Add state estimates as variables, use OAK-D measurement as initial position value
        initial.insert(state_symbol,gtsam::Vector6(det->detections[0].position.x,det->detections[0].position.y,det->detections[0].position.z,0.,0.,0.));

        // LOOP CONTROL - TODO remove after testing
        ++ii;
        last_state_symbol = state_symbol;
        last_t = t;

        if (ii>n_meas) {break;}
    }

    initial.insert(sensor_noise_symbol,oakdPosNoise);
    initial.insert(proc_noise_symbol,procNoise);

    bag.close();

    // Optimize graph
    graph.print("Graph: \n");
    graph.saveGraph("oakd_cal_init.dot",initial);
    gtsam::Values final = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
    graph.saveGraph("oakd_cal_final.dot",final);

    // Also print out to console
    initial.print("Initial result:\n");
    final.print("Final result:\n");

    // Print error to console
    std::cout << "Initial error: " << graph.error(initial) << std::endl;
    std::cout << "Final error: " << graph.error(final) << std::endl;

    return 0;
}
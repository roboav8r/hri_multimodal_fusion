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
#include <gtsam_unstable/dynamics/PoseRTV.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "depthai_ros_msgs/SpatialDetectionArray.h"

#include "hri_factors.hpp"

// Constants
std::string package_path = ros::package::getPath("hri_multimodal_fusion");
std::string bag_filepath{"/data/test.bag"}; // TODO get as input arg
std::vector<std::string> topics;


int main() {

    // Create graph and values to optimize
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Initial object pose estimate
    gtsam::Rot3 priorOrientation = gtsam::Rot3::Identity();
    // gtsam::Point3 priorPosition(0.0, 0.0, 0.0);
    // gtsam::Pose3 priorPose(priorOrientation, priorPosition);
    // Eigen::Matrix<double,4,4> priorCov;
    // TODO make this multiagent for the number of objects in label

    // OAK-D Sensor noise model
    auto oakdPosNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.1));

    // Motion model / odometry factor between successive poses
    gtsam::Rot3 fixedOrientation = gtsam::Rot3::Identity();
    gtsam::Point3 fixedPosition(0., 0., 0.);
    gtsam::Pose3 fixedMotionModel(fixedOrientation,fixedPosition);
    auto fixedMotionNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(0.00001,0.00001,0.00001,0.00001,0.00001,0.00001));

    // Velocity model
    gtsam::Vector3 velVel(0.,0.,0.);
    gtsam::PoseRTV priorVelocity(fixedOrientation,fixedPosition,velVel);
    auto velocityNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(0.00001,0.00001,0.00001));

    // Read bag file
    rosbag::Bag bag;
    bag.open(package_path + bag_filepath, rosbag::bagmode::Read);
    topics.push_back(std::string("yolov4_publisher/color/yolov4_Spatial_detections"));
    rosbag::View view(bag, rosbag::TopicQuery(topics)); // TODO replace this with rosbag::View(bag) to get all topics

    // Iterate through bagfile messages
    int n_meas = 5; // TODO replace n_meas with size of view 
    gtsam::Symbol last_pose_symbol, pose_symbol, meas_symbol, vel_symbol, last_vel_symbol;
    ros::Time t, last_t;
    ros::Duration dt;
    for(unsigned short ii=1; rosbag::MessageInstance const m : view)
    {
        // Get message data
        depthai_ros_msgs::SpatialDetectionArray::ConstPtr det = m.instantiate<depthai_ros_msgs::SpatialDetectionArray>();
        std::cout << *det << std::endl;
        t = det->header.stamp;
        std::cout << t << std::endl;

        // Create symbols for measurements, position, and velocity
        pose_symbol = gtsam::Symbol('x',ii);
        vel_symbol = gtsam::Symbol('v',ii);
        meas_symbol = gtsam::Symbol('z',ii);

        // Add sensor measurements as update factors
        graph.emplace_shared<OakDInferenceFactor>(pose_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z, oakdPosNoise);

        // Add motion model as factors
        if (ii==1) { // if this is the first node, add a prior factor
            //graph.add(gtsam::PriorFactor<gtsam::Pose3>(1,priorPose,priorCov));
        } 
        else 
        {
            // Add position prediction factor / state transition
            //graph.add(gtsam::BetweenFactor<gtsam::Pose3>(last_pose_symbol,pose_symbol,fixedMotionModel,fixedMotionNoise));
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_pose_symbol,pose_symbol,fixedMotionModel,fixedMotionNoise);

            // Add velocity prediction factor / constraint
            dt = t - last_t;
            graph.emplace_shared<gtsam::VelocityConstraint>(last_vel_symbol,vel_symbol,dt.toSec());
            //graph.add(gtsam::VelocityConstraint(last_vel_symbol,vel_symbol,))

        }

        // Add state estimates as variables, use measurement as initial value
        initial.insert(pose_symbol,gtsam::Pose3(priorOrientation,gtsam::Point3(det->detections[0].position.x,det->detections[0].position.y,det->detections[0].position.z)));

        // Add velocity variable at this timestep
        initial.insert(vel_symbol,priorVelocity);


        // LOOP CONTROL - TODO remove after testing
        ++ii;
        last_pose_symbol = pose_symbol;
        last_vel_symbol = vel_symbol;
        last_t = t;

        if (ii>n_meas) {break;}
    }

    bag.close();

    // Optimize graph
    graph.print("Graph: \n");
    graph.saveGraph("oakd_cal_init.dot",initial);
    gtsam::Values final = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
    //std::ofstream os("oakd_cal.dot");
    graph.saveGraph("oakd_cal_final.dot",final);

    // Also print out to console
    initial.print("Initial result:\n");
    final.print("Final result:\n");



    return 0;
}
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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "depthai_ros_msgs/SpatialDetectionArray.h"

// #include <boost/foreach.hpp>
// #define foreach BOOST_FOREACH

// Constants
std::string package_path = ros::package::getPath("hri_multimodal_fusion");
std::string bag_filepath{"/data/test.bag"}; // TODO get as input arg
std::vector<std::string> topics;


// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDInferenceFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactor> shared_ptr;

    OakDInferenceFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Pose3& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T.x() - mx_, T.y() - my_, T.z() - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactor(*this)));}
};


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

    // Read bag file
    rosbag::Bag bag;
    bag.open(package_path + bag_filepath, rosbag::bagmode::Read);
    topics.push_back(std::string("yolov4_publisher/color/yolov4_Spatial_detections"));
    rosbag::View view(bag, rosbag::TopicQuery(topics)); // TODO replace this with rosbag::View(bag) to get all topics

    // Iterate through bagfile messages
    int n_meas = 5; // TODO replace n_meas with size of view 
    gtsam::Symbol last_state_symbol, state_symbol, meas_symbol;
    for(unsigned short ii=1; rosbag::MessageInstance const m : view)
    {
        // Get message data
        depthai_ros_msgs::SpatialDetectionArray::ConstPtr det = m.instantiate<depthai_ros_msgs::SpatialDetectionArray>();
        std::cout << *det << std::endl;
        std::cout << det->header.stamp << std::endl;

        // Create symbols for measurements and position
        state_symbol = gtsam::Symbol('x',ii);
        meas_symbol = gtsam::Symbol('z',ii);
        
        // Add sensor measurements as factors
        graph.emplace_shared<OakDInferenceFactor>(state_symbol,det->detections[0].position.x, det->detections[0].position.y, det->detections[0].position.z, oakdPosNoise);

        // Add motion model as factors
        if (ii==1) { // if this is the first node, add a prior factor
            //graph.add(gtsam::PriorFactor<gtsam::Pose3>(1,priorPose,priorCov));
        } 
        else // add a motion factor
        {
            //graph.add(gtsam::BetweenFactor<gtsam::Pose3>(last_state_symbol,state_symbol,fixedMotionModel,fixedMotionNoise));
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_state_symbol,state_symbol,fixedMotionModel,fixedMotionNoise);
        }

        // Add state estimates as variables, use measurement as initial value
        initial.insert(state_symbol,gtsam::Pose3(priorOrientation,gtsam::Point3(det->detections[0].position.x,det->detections[0].position.y,det->detections[0].position.z)));

        // LOOP CONTROL - TODO remove after testing
        ++ii;
        last_state_symbol = state_symbol;
        if (ii>n_meas) {break;}
    }

    bag.close();

    // Optimize graph
    graph.print("Graph: \n");
    gtsam::Values final = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
    std::ofstream os("oakd_cal.dot");
    graph.saveGraph("oakd_cal.dot",final);

    // Also print out to console
    initial.print("Initial result:\n");
    final.print("Final result:\n");



    return 0;
}
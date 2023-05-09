#include <iostream>
#include <fstream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// #include <boost/foreach.hpp>
// #define foreach BOOST_FOREACH

// Constants
std::string package_path = ros::package::getPath("hri_multimodal_fusion");
std::string bag_filepath{"/data/test.bag"}; // TODO get as input arg
std::vector<std::string> topics;

int main() {

    // Create graph
    gtsam::NonlinearFactorGraph graph;

    // Add factors to graph from bag file
    rosbag::Bag bag;
    bag.open(package_path + bag_filepath, rosbag::bagmode::Read);
    //bag.open("bag_filepath", rosbag::bagmode::Read);
    
    topics.push_back(std::string("yolov4_publisher/color/yolov4_Spatial_detections"));
    // topics.push_back(std::string("yolov4_publisher/color/image"));

    rosbag::View view(bag, rosbag::TopicQuery(topics)); // TODO replace this with rosbag::View(bag) to get all topics

    // Iterate through messages in bag
    int n_meas = 10; // TODO replace n_meas with size of view 
    int view_size = view.size();
    std::cout << "View has size: " << view_size << std::endl;
    // for (unsigned int ii = 0; ii < n_meas; ii++)
    // {

    // }

    bag.close();

    // Add labeled factors to graph




    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
    // std::ofstream os("Pose2SLAMExample.dot");
    // graph.saveGraph(os, result);

    // Also print out to console
    // graph.saveGraph(std::cout, result);


    return 0;
}
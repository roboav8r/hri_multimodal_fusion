#include <sstream>
#include <fstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "rosbag/bag.h"

#include "depthai_ros_msgs/SpatialDetectionArray.h"
#include "depthai_ros_msgs/SpatialDetection.h"

#include "hri_multimodal_fusion/LabelBagData.h"

/*
DEFINE CONSTANTS/INPUTS
*/
const std::vector<std::string> class_labels = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};


std::string detection_topic{"yolov4_publisher/color/yolov4_Spatial_detections"};
std::string image_topic{"yolov4_publisher/color/image"};
static const std::string OAKD_WINDOW = "OAK-D Detections";

std::string package_path = ros::package::getPath("hri_multimodal_fusion");
// std::string topics = detection_topic + " " + image_topic;
std::string topics = detection_topic;

bool bagAndLabelData(hri_multimodal_fusion::LabelBagData::Request &req,
                        hri_multimodal_fusion::LabelBagData::Response &resp)
{
    // Generate and send rosbag command
    std::stringstream bagCommand;
    bagCommand << "rosbag record -O ";
    bagCommand << package_path << "/" << req.filepath;
    bagCommand << " --duration=" << (uint)req.duration_sec << "s ";
    bagCommand << topics;

    ROS_INFO("rosbag command: %s", bagCommand.str().c_str());
    std::system(bagCommand.str().c_str());

    // Generate and save label file associated with rosbag
    std::ofstream labelFile;
    labelFile.open(package_path + "/" + req.filepath + ".yaml");

    // Write label data to file
    for (auto& label : req.label_data.labels)
    {
        // TODO - format this to make it easier to put into graph form
        labelFile << std::to_string(label.size) << std::endl;
        labelFile << label.label << std::endl;
        labelFile << label.activity << std::endl;
        labelFile << std::to_string(label.motiontype) << std::endl;
        labelFile << std::endl;
    }

    labelFile.close();

    return true;
}

/*
DEFINE OAK-D TRAINER CLASS
*/
class TrainerNode
{
    private:
    /* 
    PRIVATE MEMBERS
    */

    // Constants
    int detection_font_size_{1};
    const int detection_font_weight_{1};
    cv::Scalar color_ = CV_RGB(0,255,255);

    // Subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_; 
    ros::Subscriber yolo_sub_;

    // Services
    ros::ServiceServer bag_and_label_srv_;

    // Private members to store incoming messages    
    depthai_ros_msgs::SpatialDetectionArray last_detection_msg_;
    sensor_msgs::Image last_img_;
    cv_bridge::CvImagePtr cv_ptr_bgr_;

    // Private member functions
    void renderVisuals_()
    {

        // Convert RGB ROS message to CV matrix
        try
        {
            cv_ptr_bgr_ = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Add bounding boxes and labels to detections
        for (int ii=0 ; ii < last_detection_msg_.detections.size(); ii++)
        {
            cv::rectangle(cv_ptr_bgr_->image, cv::Point(last_detection_msg_.detections[ii].bbox.center.x + last_detection_msg_.detections[ii].bbox.size_x/2, last_detection_msg_.detections[ii].bbox.center.y + last_detection_msg_.detections[ii].bbox.size_y/2), cv::Point(last_detection_msg_.detections[ii].bbox.center.x - last_detection_msg_.detections[ii].bbox.size_x/2, last_detection_msg_.detections[ii].bbox.center.y - last_detection_msg_.detections[ii].bbox.size_y/2), color_, detection_font_weight_);
            cv::putText(cv_ptr_bgr_->image, class_labels[((int)last_detection_msg_.detections[ii].results[0].id)] + " @ "+std::to_string(last_detection_msg_.detections[ii].position.x) + ","+ std::to_string(last_detection_msg_.detections[ii].position.y) + "," + std::to_string(last_detection_msg_.detections[ii].position.z), cv::Point(last_detection_msg_.detections[ii].bbox.center.x - last_detection_msg_.detections[ii].bbox.size_x/2, last_detection_msg_.detections[ii].bbox.center.y - last_detection_msg_.detections[ii].bbox.size_y/2 -10),cv::FONT_HERSHEY_PLAIN, detection_font_size_,color_, detection_font_weight_);
        }
        
        //cv_ptr_bgr_ = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
        cv::imshow(OAKD_WINDOW, cv_ptr_bgr_->image);
        cv::waitKey(1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        last_img_ = (*msg);
    }; // Image callback

    void yoloDetectionCallback(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& msg)
    {
        last_detection_msg_ = (*msg);
        renderVisuals_();
    }; // YOLO detection callback

    public:
    // Default Constructor
    TrainerNode() : it_(nh_)
    {

        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe(image_topic, 10, &TrainerNode::imageCallback, this);
        yolo_sub_ = nh_.subscribe(detection_topic, 10, &TrainerNode::yoloDetectionCallback, this);

        // Start service to bag and label camera data
        bag_and_label_srv_ = nh_.advertiseService("bag_and_label_sensor_data", bagAndLabelData);

        cv::namedWindow(OAKD_WINDOW);
    }

    // Default Destructor
    ~TrainerNode()
    {
        cv::destroyWindow(OAKD_WINDOW);
    }
    
};


/*
MAIN LOOP
*/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "perception_train_node");

    TrainerNode trainer_node;

    ros::spin();
    return 0;
}


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "depthai_ros_msgs/SpatialDetectionArray.h"
#include "depthai_ros_msgs/SpatialDetection.h"



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

/*
DEFINE OAK-D TRAINER CLASS
*/
class OakDTrainerNode
{
    private:
    /* 
    PRIVATE MEMBERS
    */

    // Subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_; 
    ros::Subscriber yolo_sub_;

    // Private members to store incoming messages    
    depthai_ros_msgs::SpatialDetectionArray last_detection_msg_;
    sensor_msgs::Image last_img_;
    cv_bridge::CvImagePtr cv_ptr_bgr_;

    // Private member functions
    void renderVisuals_()
    {

        // Convert ROS message to CV matrix
        try
        {
            cv_ptr_bgr_ = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //cv_ptr_bgr_ = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
        cv::imshow(OAKD_WINDOW, cv_ptr_bgr_->image);
        cv::waitKey(1);
    }

    public:
    // Default Constructor
    OakDTrainerNode() : it_(nh_)
    {

        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe(image_topic, 10, &OakDTrainerNode::imageCallback, this);
        yolo_sub_ = nh_.subscribe(detection_topic, 10, &OakDTrainerNode::yoloDetectionCallback, this);

        cv::namedWindow(OAKD_WINDOW);
    }

    // Default Destructor
    ~OakDTrainerNode()
    {
        cv::destroyWindow(OAKD_WINDOW);
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        last_img_ = (*msg);
        // renderVisuals_();
    }; // Image callback

    void yoloDetectionCallback(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& msg)
    {
        last_detection_msg_ = (*msg);
        renderVisuals_();
    }; // YOLO detection callback


};


/*
MAIN LOOP
*/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "oakd_train_node");

    OakDTrainerNode oakd_trainer_node;

    ros::spin();
    return 0;
}


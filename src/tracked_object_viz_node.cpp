#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include "hri_multimodal_fusion/TrackedObject.h"

class TrackedObjectVisualizer
{
    public:
    TrackedObjectVisualizer(ros::NodeHandle nh) : nh_(nh) 
    {
        vizPub_ = nh_.advertise<visualization_msgs::MarkerArray>("tracked_object_markers",10);
    }

    void Callback (const hri_multimodal_fusion::TrackedObject::ConstPtr& msg)
    {
        ROS_INFO("Received message");

        // Initialize message
        markerArrayMsg_ = visualization_msgs::MarkerArray();
        markerMsg_ = visualization_msgs::Marker();
        markerMsg_.header = msg->header;
        markerMsg_.color.a = 1.0; 
        markerMsg_.color.r = 0.0;
        markerMsg_.color.g = 1.0;
        markerMsg_.color.b = 0.0;
        geometry_msgs::Point endPoint, labelOffset;

        // Form messages
        // Bounding Box
        markerMsg_.id=0;
        markerMsg_.type = visualization_msgs::Marker::LINE_LIST;
        markerMsg_.action = visualization_msgs::Marker::ADD;
        markerMsg_.scale.x = 0.025;
        geometry_msgs::Point frontTopLeft,frontTopRight, frontBottomLeft, frontBottomRight, backTopLeft, backTopRight, backBottomLeft, backBottomRight;
        double front, back, left, right, top, bottom;
        front = msg->pose.pose.position.z - sqrt(msg->pose.covariance[14])/100;
        back = msg->pose.pose.position.z + sqrt(msg->pose.covariance[14])/100;
        left = msg->pose.pose.position.x - sqrt(msg->pose.covariance[0])/100;
        right = msg->pose.pose.position.x + sqrt(msg->pose.covariance[0])/100;
        bottom = msg->pose.pose.position.y - sqrt(msg->pose.covariance[7])/100;
        top = msg->pose.pose.position.y + sqrt(msg->pose.covariance[7])/100;
        frontTopLeft.x = left; frontTopLeft.y = top; frontTopLeft.z = front;
        frontTopRight.x = right; frontTopRight.y = top; frontTopRight.z = front;
        frontBottomLeft.x = left; frontBottomLeft.y = bottom; frontBottomLeft.z = front;
        frontBottomRight.x = right; frontBottomRight.y = bottom; frontBottomRight.z = front;
        backTopLeft.x = left; backTopLeft.y = top; backTopLeft.z = back;
        backTopRight.x = right; backTopRight.y = top; backTopRight.z = back;
        backBottomLeft.x = left; backBottomLeft.y = bottom; backBottomLeft.z = back;
        backBottomRight.x = right; backBottomRight.y = bottom; backBottomRight.z = back;        
        markerMsg_.points.push_back(frontTopLeft); markerMsg_.points.push_back(frontTopRight);
        markerMsg_.points.push_back(frontTopLeft); markerMsg_.points.push_back(frontBottomLeft);
        markerMsg_.points.push_back(frontBottomLeft); markerMsg_.points.push_back(frontBottomRight);
        markerMsg_.points.push_back(frontTopRight); markerMsg_.points.push_back(frontBottomRight);
        markerMsg_.points.push_back(backTopLeft); markerMsg_.points.push_back(backTopRight);
        markerMsg_.points.push_back(backTopLeft); markerMsg_.points.push_back(backBottomLeft);
        markerMsg_.points.push_back(backBottomLeft); markerMsg_.points.push_back(backBottomRight);
        markerMsg_.points.push_back(backTopRight); markerMsg_.points.push_back(backBottomRight);
        markerMsg_.points.push_back(frontBottomLeft); markerMsg_.points.push_back(backBottomLeft);
        markerMsg_.points.push_back(frontTopLeft); markerMsg_.points.push_back(backTopLeft);
        markerMsg_.points.push_back(frontBottomRight); markerMsg_.points.push_back(backBottomRight);
        markerMsg_.points.push_back(frontTopRight); markerMsg_.points.push_back(backTopRight);
        markerArrayMsg_.markers.push_back(markerMsg_);

        // Velocity Arrow
        markerMsg_.id=1;
        markerMsg_.type = visualization_msgs::Marker::ARROW;
        markerMsg_.action = visualization_msgs::Marker::ADD;
        markerMsg_.scale.x = 0.1;
        markerMsg_.scale.y = 0.125;
        markerMsg_.scale.z = 0.125;
        markerMsg_.points.push_back(msg->pose.pose.position); // Add start point at position
        endPoint.x = msg->pose.pose.position.x + msg->twist.twist.linear.x;
        endPoint.y = msg->pose.pose.position.y + msg->twist.twist.linear.y;
        endPoint.z = msg->pose.pose.position.z + msg->twist.twist.linear.z;
        markerMsg_.points.push_back(endPoint); // Add start point at position
        markerArrayMsg_.markers.push_back(markerMsg_);
        

        // Activity label
        markerMsg_.id=2;
        markerMsg_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markerMsg_.action = visualization_msgs::Marker::ADD;
        markerMsg_.scale.z = .2;
        labelOffset.x = 0;
        labelOffset.y = -0.5;
        labelOffset.z = 0;
        markerMsg_.pose.position.x = msg->pose.pose.position.x + labelOffset.x;
        markerMsg_.pose.position.y = msg->pose.pose.position.y + labelOffset.y;
        markerMsg_.pose.position.z = msg->pose.pose.position.z + labelOffset.z;
        markerMsg_.text = "Activity: " + msg->activity + " (" + std::to_string(msg->activity_confidence) + "%) ";
        markerArrayMsg_.markers.push_back(markerMsg_);
        
        // Publish message
        vizPub_.publish(markerArrayMsg_);
    }

    private:
    // ROS/loop control member variables
    ros::NodeHandle nh_;
    ros::Time last_t_;

    // Output
    visualization_msgs::Marker markerMsg_;
    visualization_msgs::MarkerArray markerArrayMsg_;
    ros::Publisher vizPub_;

};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "tracked_object_viz_node");
  ros::NodeHandle nh;

  // Create a visualizer object
  TrackedObjectVisualizer viz(nh);

  // Create a subscriber
  ros::Subscriber sub = nh.subscribe("tracks", 10, &TrackedObjectVisualizer::Callback, &viz);

  // Spin the node
  ros::spin();

  return 0;
}
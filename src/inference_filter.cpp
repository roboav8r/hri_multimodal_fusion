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


class InferenceFilter
{
    public:
    // Constructor
    InferenceFilter(){};

    // Accessors
    gtsam::KalmanFilter::State State() {
        return this->state_;
    };

    gtsam::SharedDiagonal TransNoise()
    {
        return this->transNoise_;
    };


    // Mutators
    void Predict()
    {
        this->state_ = kf_.predict(this->state_,this->transModel_,this->inputModel_,this->input_,this->transNoise_);
    }

    void TransModel(double dt)
    {
        transModel_(0,3) = dt;
        transModel_(1,4) = dt;
        transModel_(2,5) = dt;
    }

    void TransNoise(double dt)
    {
        this->transNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(transNoiseCoeffs_(0)*pow(dt,2),
                                                          transNoiseCoeffs_(1)*pow(dt,2),
                                                          transNoiseCoeffs_(2)*pow(dt,2),
                                                          transNoiseCoeffs_(3)*dt,
                                                          transNoiseCoeffs_(4)*dt,
                                                          transNoiseCoeffs_(5)*dt));
    }

    void Update()
    {
        this->state_ = kf_.update(this->state_, this->oakDMeasModel_, this->oakDMeas_, this->oakDNoise_);
    }

    void OakDUpdate(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& msg)
    {
        ROS_INFO("Got detection msg");

        if (this->initialized_) {

            // Compute time since last estimate
            this->last_t_ = t_;
            this->t_ = msg->header.stamp;
            this->dt_ = (this->t_ - this->last_t_).toSec();
            std::cout<<this->dt_<<std::endl;

            // Compute state transition model based on dt_
            TransModel(this->dt_);
            TransNoise(this->dt_);

            // Do a prediction/propagation step
            Predict();

            // If measurement is available, do an update
            if (msg->detections.size() !=0) {

                for (depthai_ros_msgs::SpatialDetection det : msg->detections) {
                    this->oakDMeas_ = {det.position.x, det.position.y, det.position.z};
                }

                Update();
            }

            // Publish/output current state
            state_->print("State");


        } else {
            ROS_INFO("Filter NOT initialized");
            
            // If a detection is available, set this as the initial estimate
            if (msg->detections.size() !=0) {

                t_ = msg->header.stamp;

                for (depthai_ros_msgs::SpatialDetection det : msg->detections) {
                    this->oakDMeas_ = {det.position.x, det.position.y, det.position.z};
                }
                // Set filter to initialized
                state_ = this->kf_.init(gtsam::Vector6(this->oakDMeas_(0),this->oakDMeas_(1),this->oakDMeas_(2),0,0,0), 
                                        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(this->oakDNoiseCoeffs_(0),this->oakDNoiseCoeffs_(1),this->oakDNoiseCoeffs_(2),this->transNoiseCoeffs_(0),this->transNoiseCoeffs_(1),this->transNoiseCoeffs_(2))));
                this->initialized_ = true;
            }
        }
    };

    private:
    // State transition models
    const gtsam::Vector6 transNoiseCoeffs_ = gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
    gtsam::SharedDiagonal transNoise_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseCoeffs_);
    gtsam::Matrix transModel_ = gtsam::Matrix::Identity(6,6); // "F" matrix
    gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6);
    gtsam::Vector6 input_ = gtsam::Vector::Zero(6);

    // GTSAM measurement models
    gtsam::Vector3 oakDMeas_;
    const gtsam::Matrix oakDMeasModel_ = (gtsam::Matrix(3,6)<< 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0).finished();
    const gtsam::Vector3 oakDNoiseCoeffs_ = gtsam::Vector3(34.0059364,25.9475303,54.9710593);
    const gtsam::SharedDiagonal oakDNoise_ = gtsam::noiseModel::Diagonal::Sigmas(oakDNoiseCoeffs_);

    
    // GTSAM filter member variables
    gtsam::KalmanFilter kf_{6, gtsam::KalmanFilter::Factorization::QR};
    gtsam::KalmanFilter::State state_;

    
    // ROS/loop control member variables
    ros::Time t_, last_t_;
    double dt_;
    bool initialized_{false};

};



int main(int argc, char **argv) {


    ros::init(argc, argv, "inference_filter");
    ros::NodeHandle nh;

    InferenceFilter filter;

    ros::Subscriber yoloSub = nh.subscribe(yoloTopic, 1, &InferenceFilter::OakDUpdate, &filter);

    ros::spin();
    return 0;

};
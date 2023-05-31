#include "state.hpp"
#include "transition_models.hpp"

class InferenceFilter
{
    public:
    // Constructor
    InferenceFilter(){};

    // Accessors
    ObjectState State() {
        return this->state_;
    };

    // Mutators
    void Predict()
    {
        //this->state_.x = kf_.predict(this->state_.x,this->transModel_,this->inputModel_,this->input_,this->transNoise_);
        this->state_.x = kf_.predict(this->state_.x,motionModel_.TransModel(),motionModel_.InputModel(),motionModel_.Input(),motionModel_.TransCov());
    }

    void Update()
    {
        this->state_.x = kf_.update(this->state_.x, this->oakDMeasModel_, this->oakDMeas_, this->oakDNoise_);
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
            motionModel_.UpdateTrans(this->dt_);

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
            state_.x->print("State");


        } else {
            ROS_INFO("Filter NOT initialized");
            
            // If a detection is available, set this as the initial estimate
            if (msg->detections.size() !=0) {

                t_ = msg->header.stamp;

                for (depthai_ros_msgs::SpatialDetection det : msg->detections) {
                    this->oakDMeas_ = {det.position.x, det.position.y, det.position.z};
                }
                // Set filter to initialized
                state_.x = this->kf_.init(gtsam::Vector6(this->oakDMeas_(0),this->oakDMeas_(1),this->oakDMeas_(2),0,0,0), 
                                        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(this->oakDNoiseCoeffs_(0),this->oakDNoiseCoeffs_(1),this->oakDNoiseCoeffs_(2),this->motionModel_.TransVar()(0),this->motionModel_.TransVar()(1),this->motionModel_.TransVar()(2))));
                this->initialized_ = true;
            }
        }
    };

    private:
    // State transition models
    ConstVelMotion motionModel_ = ConstVelMotion(gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441));

    // GTSAM measurement models
    gtsam::Vector3 oakDMeas_;
    const gtsam::Matrix oakDMeasModel_ = (gtsam::Matrix(3,6)<< 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0).finished();
    const gtsam::Vector3 oakDNoiseCoeffs_ = gtsam::Vector3(34.0059364,25.9475303,54.9710593);
    const gtsam::SharedDiagonal oakDNoise_ = gtsam::noiseModel::Diagonal::Sigmas(oakDNoiseCoeffs_);
    
    // GTSAM filter member variables
    gtsam::KalmanFilter kf_{6, gtsam::KalmanFilter::Factorization::QR};
    ObjectState state_;

    // ROS/loop control member variables
    ros::Time t_, last_t_;
    double dt_;
    bool initialized_{false};

};
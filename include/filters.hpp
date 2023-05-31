#include "state.hpp"
#include "transition_models.hpp"
#include "obs_models.hpp"

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
        this->state_.x = kf_.predict(this->state_.x,this->motionModel_.TransModel(),this->motionModel_.InputModel(),this->motionModel_.Input(),this->motionModel_.TransCov());
    }

    void Update()
    {
        this->state_.x = kf_.update(this->state_.x, this->oakDSensor_.MeasModel(), this->oakDMeas_, this->oakDSensor_.NoiseCov());
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
                                        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(this->oakDSensor_.NoiseVar()(0),this->oakDSensor_.NoiseVar()(1),this->oakDSensor_.NoiseVar()(2),this->motionModel_.TransVar()(0),this->motionModel_.TransVar()(1),this->motionModel_.TransVar()(2))));
                this->initialized_ = true;
            }
        }
    };

    private:
    // State transition models
    ConstVelMotion motionModel_ = ConstVelMotion(gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441));

    // GTSAM measurement models
    gtsam::Vector3 oakDMeas_;
    OakDSensor oakDSensor_ = OakDSensor(gtsam::Vector3(34.0059364,25.9475303,54.9710593));
    
    // GTSAM filter member variables
    gtsam::KalmanFilter kf_{6, gtsam::KalmanFilter::Factorization::QR};
    ObjectState state_;

    // ROS/loop control member variables
    ros::Time t_, last_t_;
    double dt_;
    bool initialized_{false};

};
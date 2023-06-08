#ifndef FILTERS_H
#define FILTERS_H

#include <ros/ros.h>

#include "state.hpp"
#include "transition_models.hpp"
#include "obs_models.hpp"

#include "hri_multimodal_fusion/TrackedObject.h"

namespace Filters {

struct FilterParams
{
    std::string FrameId;
    int MaxGaussians;
    double PruneThresh;
    double MergeThresh;
    double PGate;
    bool EnableGating;
};

// Helper function to extract filter params from ros param path
FilterParams ExtractFilterParams(std::string param_ns, ros::NodeHandle n)
{
    FilterParams params;
    n.getParam(param_ns + "frame_id", params.FrameId);
    n.getParam(param_ns + "max_gaussians", params.MaxGaussians);
    n.getParam(param_ns + "prune_threshold", params.PruneThresh);
    n.getParam(param_ns + "merge_threshold", params.MergeThresh);
    n.getParam(param_ns + "p_gate", params.PGate);
    n.getParam(param_ns + "gating", params.EnableGating);

    return params;
};

// Helper function to convert GTSAM output to ROS message
void FormatObjectMsg(gtsam::KalmanFilter::State& state, hri_multimodal_fusion::TrackedObject& msg)
{
    // TODO: Get most likely activity index
    int8_t motionModelInd = 0;
    
    // Update pose & covariance
    msg.pose.pose.position.x=state->mean()[0];
    msg.pose.pose.position.y=state->mean()[1];
    msg.pose.pose.position.z=state->mean()[2];
    msg.pose.pose.orientation.w=1;
    msg.pose.covariance[0]=state->covariance()(0,0);
    msg.pose.covariance[7]=state->covariance()(1,1);
    msg.pose.covariance[14]=state->covariance()(2,2);


    // Update velocity
    msg.twist.twist.linear.x=state->mean()[3];
    msg.twist.twist.linear.y=state->mean()[4];
    msg.twist.twist.linear.z=state->mean()[5];
    msg.twist.covariance[0]=state->covariance()(3,3); 
    msg.twist.covariance[7]=state->covariance()(4,4);
    msg.twist.covariance[14]=state->covariance()(5,5);

    // TODO add activity as categorical distribution
    
}

class InferenceFilter
{
    public:
    // // Default Constructor with parameters
    // InferenceFilter(FilterParams par, Sensors::Clutter3D clutter_model) 
    //     : params_(par), clutterModel_(clutter_model) {};

    // // Construct with motion model
    // // TODO update for multiple motion models
    // InferenceFilter(FilterParams par, TransitionModels::ConstVelMotion cv_mot, Sensors::Clutter3D clutter_model) 
    //     : params_(par), motionModel_(cv_mot), clutterModel_(clutter_model) {};

    // Construct with motion and obs models
    // TODO update for multiple motion models
    InferenceFilter(ros::NodeHandle nh, FilterParams filt_par, TransitionModels::TransModelParams trans_par, Sensors::OakDSensor oak_d, Sensors::Clutter3D clutter_model) 
        : nh_(nh), filtParams_(filt_par), transModelParams_(trans_par), state_(trans_par.nModels), oakDSensor_(oak_d), clutterModel_(clutter_model) 
        {
            trackPub_ = nh_.advertise<hri_multimodal_fusion::TrackedObject>("tracks",10);
            objectMsg_.header.frame_id = filtParams_.FrameId;
        };


    // Accessors
    ObjectState State() {
        return this->state_;
    };

    // Mutators
    void Predict()
    {
        // Predict for multiple models
        for (size_t ii=0; ii< this->transModelParams_.nModels; ++ii) 
        {

            this->state_.Spatial[ii] = kfs_[ii].predict(this->state_.Spatial[ii],
                                                        this->transModelParams_.SpatialTransModels[ii].TransModel(),
                                                        this->transModelParams_.SpatialTransModels[ii].InputModel(),
                                                        this->transModelParams_.SpatialTransModels[ii].Input(),
                                                        this->transModelParams_.SpatialTransModels[ii].TransCov());
            this->kfs_[ii].print("Predicted state for model " + ii);

        };
        // TODO predict activity state
    }

    void Update()
    {
        // Update spatial state for multiple models
                for (size_t ii=0; ii< this->transModelParams_.nModels; ++ii) 
        {
            std::cout << "Spatial state " << ii << ": " << std::endl;


            this->state_.Spatial[ii] = kfs_[ii].update(this->state_.Spatial[ii],
                                                       this->oakDSensor_.MeasModel(), 
                                                       this->oakDMeas_, 
                                                       this->oakDSensor_.NoiseCov());
            // auto loopstate = *(this->state_.Spatial[ii]);
            // loopstate.print();

        };
        
        // TODO update activity state
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

            // Update state transition model based on dt_
            for (size_t ii =0; ii< this->transModelParams_.nModels; ++ii) {
                this->transModelParams_.SpatialTransModels[ii].UpdateTrans(this->dt_);
            };

            // Do a prediction/propagation step
            Predict();

            std::cout << "Predict done" << std::endl;

            // If measurement is available, do an update
            if (msg->detections.size() !=0) {

                for (depthai_ros_msgs::SpatialDetection det : msg->detections) {
                    this->oakDMeas_ = {det.position.x, det.position.y, det.position.z};
                }

                Update();
            }

            std::cout << "Update done" << std::endl;

            // TODO find maximum likelihood index
            int8_t motionModelInd = 0;

            // Publish/output current state
            // TODO only publish most likely state
            // state_.Spatial[motionModelInd]->print("State");
            // FormatObjectMsg(state_.Spatial[motionModelInd], objectMsg_);
            // objectMsg_.header.stamp = this->t_;
            // trackPub_.publish(objectMsg_);

            std::cout << "Publishing done" << std::endl;


        } else {
            ROS_INFO("Filter NOT initialized");
            
            // If a detection is available, set this as the initial estimate
            if (msg->detections.size() !=0) {

                t_ = msg->header.stamp;

                for (depthai_ros_msgs::SpatialDetection det : msg->detections) {
                    this->oakDMeas_ = {det.position.x, det.position.y, det.position.z};
                }
                
                // Generate a KF object and a prior for each motion model
                std::cout << "Initializing Kalman Filters" << std::endl;
                for (size_t ii =0; ii< this->transModelParams_.nModels; ++ii) {

                    this->kfs_.push_back(gtsam::KalmanFilter(6, gtsam::KalmanFilter::Factorization::QR));

                    gtsam::Vector6 variance(this->oakDSensor_.NoiseVar()(0),
                                        this->oakDSensor_.NoiseVar()(1),
                                        this->oakDSensor_.NoiseVar()(2),
                                        this->transModelParams_.SpatialTransModels[ii].TransVar()(3),
                                        this->transModelParams_.SpatialTransModels[ii].TransVar()(4),
                                        this->transModelParams_.SpatialTransModels[ii].TransVar()(5));
                    // std::cout << ii << std::endl;
                    // std::cout << variance << std::endl;

                    
                    this->state_.Spatial[ii] = this->kfs_[ii].init(
                        gtsam::Vector6(this->oakDMeas_(0),this->oakDMeas_(1),this->oakDMeas_(2),0,0,0), 
                        gtsam::noiseModel::Diagonal::Sigmas(variance));

                    // std::cout << this->transModelParams_.SpatialTransModels[ii].TransCov()->sigmas() << std::endl;
                    this->kfs_[ii].print();
                };

                this->initialized_ = true;
            }
        }
    };

    private:

    // GTSAM filter member variables
    std::vector<gtsam::KalmanFilter> kfs_;
    ObjectState state_;
    FilterParams filtParams_;
    Sensors::Clutter3D clutterModel_;

    // State transition models
    TransitionModels::TransModelParams transModelParams_;

    // GTSAM measurement models
    gtsam::Vector3 oakDMeas_;
    Sensors::OakDSensor oakDSensor_ = Sensors::OakDSensor(gtsam::Vector3(34.0059364,25.9475303,54.9710593));

    // ROS/loop control member variables
    ros::NodeHandle nh_;
    ros::Time t_, last_t_;
    double dt_;
    bool initialized_{false};

    // Output
    hri_multimodal_fusion::TrackedObject objectMsg_;
    ros::Publisher trackPub_;

};

}; // Filters namespace

#endif
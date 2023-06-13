#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>
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

// Compute likelihood

double Likelihood(gtsam::Vector z, gtsam::Matrix C, gtsam::Vector x, gtsam::Matrix R)
{
    gtsam::Matrix err = z - C*x;
    return exp((-0.5*(err.transpose()*R*err)).mean()); 
}

// Helper function to convert GTSAM output to ROS message
void FormatObjectMsg(gtsam::KalmanFilter::State& state, hri_multimodal_fusion::TrackedObject& msg)
{
    
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
    // Construct with motion and obs models
    InferenceFilter(ros::NodeHandle nh, FilterParams filt_par, TransitionModels::TransModelParams trans_par, Sensors::OakDSensor oak_d, Sensors::Clutter3D clutter_model) 
        : nh_(nh), filtParams_(filt_par), transModelParams_(trans_par), state_(trans_par.nModels), oakDSensor_(oak_d), clutterModel_(clutter_model) 
        {
            trackPub_ = nh_.advertise<hri_multimodal_fusion::TrackedObject>("tracks",10);
            objectMsg_.header.frame_id = filtParams_.FrameId;
            for (int ii=0; ii<transModelParams_.nModels; ii++)
            {
                objectMsg_.activity_labels.push_back(transModelParams_.MotionLabels[ii]);
            }
            objectMsg_.activity_confidences.resize(transModelParams_.nModels);
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
                                                        this->transModelParams_.SpatialTrans[ii].TransModel(),
                                                        this->transModelParams_.SpatialTrans[ii].InputModel(),
                                                        this->transModelParams_.SpatialTrans[ii].Input(),
                                                        this->transModelParams_.SpatialTrans[ii].TransCov());

        };

        // Predict activity state
        this->state_.Motion = this->transModelParams_.MotionTrans*this->state_.Motion;
        
    }

    void Update()
    {
        // Update spatial state for multiple models
        for (size_t ii=0; ii< this->transModelParams_.nModels; ++ii) 
        {
            // Compute Likelihood of this measurement with the predicted model
            this->state_.Likelihood[ii] = Likelihood(this->oakDMeas_, this->oakDSensor_.MeasModel(), this->state_.Spatial[ii]->mean(), this->oakDSensor_.NoiseCov()->R());

            // Perform update step
            this->state_.Spatial[ii] = kfs_[ii].update(this->state_.Spatial[ii],
                                                       this->oakDSensor_.MeasModel(), 
                                                       this->oakDMeas_, 
                                                       this->oakDSensor_.NoiseCov());

        };
        
        // Update motion state based on likelihood, then normalize Motion probabilities
        for (size_t ii=0; ii< this->transModelParams_.nModels; ++ii) 
        { this->state_.Motion(ii,0) *= this->state_.Likelihood(ii,0);} 
        this->state_.Motion /= this->state_.Motion.sum();
    }

    void OakDUpdate(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& msg)
    {
        ROS_INFO("Got detection msg");

        if (this->initialized_) {

            // Compute time since last estimate
            this->last_t_ = t_;
            this->t_ = msg->header.stamp;
            this->dt_ = (this->t_ - this->last_t_).toSec();

            // Update state transition model based on dt_
            for (size_t ii =0; ii< this->transModelParams_.nModels; ++ii) {
                this->transModelParams_.SpatialTrans[ii].UpdateTrans(this->dt_);
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

            // Find maximum likelihood index
            int8_t motionModelInd = 0;
            float motionConf = 0.0;
            for (size_t ii =0; ii< this->transModelParams_.nModels; ++ii) {
                objectMsg_.activity_confidences[ii] = this->state_.Motion(ii,0);
                if (this->state_.Motion(ii,0) > motionConf)
                {
                    motionConf = this->state_.Motion(ii,0);
                    motionModelInd = ii;
                }
            };

            // Publish/output current state
            // state_.Spatial[motionModelInd]->print("State");
            FormatObjectMsg(state_.Spatial[motionModelInd], objectMsg_);
            objectMsg_.header.stamp = this->t_;
            objectMsg_.activity = this->transModelParams_.MotionLabels[motionModelInd];
            objectMsg_.activity_confidence = motionConf;
            trackPub_.publish(objectMsg_);

            // std::cout << "Publishing done" << std::endl;

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
                                        this->transModelParams_.SpatialTrans[ii].TransVar()(3),
                                        this->transModelParams_.SpatialTrans[ii].TransVar()(4),
                                        this->transModelParams_.SpatialTrans[ii].TransVar()(5));

                    
                    this->state_.Spatial[ii] = this->kfs_[ii].init(
                        gtsam::Vector6(this->oakDMeas_(0),this->oakDMeas_(1),this->oakDMeas_(2),0,0,0), 
                        gtsam::noiseModel::Diagonal::Sigmas(variance));
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
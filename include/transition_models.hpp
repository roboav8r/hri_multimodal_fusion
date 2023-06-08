#ifndef TRANS_MODELS_H
#define TRANS_MODELS_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DiscreteDistribution.h>

namespace TransitionModels {

/*
Forward Declarations
*/
enum SpatialTransType
{
    const_pos,
    const_vel
};

struct TransModelParams;

TransModelParams ExtractTransModelParams(std::string, ros::NodeHandle&);
void PrintTransModelParams(TransModelParams);

class SpatialTransition
{
    public:
    // Construct from yaml input
    SpatialTransition(SpatialTransType, std::vector<double>);

    // Accessors
    SpatialTransType ModelType();
    gtsam::Matrix TransModel();
    gtsam::Vector6 TransVar();
    gtsam::SharedDiagonal TransCov();
    gtsam::Matrix InputModel();
    gtsam::Vector6 Input();

    // Mutators
    void TransModel(double);
    void TransCov(double);
    void UpdateTrans(double);

    private:
    SpatialTransType type_;
    gtsam::Matrix transModel_ = gtsam::Matrix::Zero(6,6); // "F" matrix
    gtsam::Vector6 transNoiseVar_;
    gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
    const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
    const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector

};

}; // TransitionModels namespace

/*
Definitions
*/

// Structure containing all transition model data
struct TransitionModels::TransModelParams
{
    size_t nModels;
    std::vector<TransitionModels::SpatialTransition> SpatialTransModels;
    std::vector<gtsam::DiscreteConditional> MotionTransModels;
    std::vector<std::string> MotionLabels;
};

// Helper function to extract transition model params from ros param path
TransitionModels::TransModelParams TransitionModels::ExtractTransModelParams(std::string param_ns, ros::NodeHandle& n)
{
    TransModelParams params;
    std::string motionLabel;
    XmlRpc::XmlRpcValue paramMap;
    std::vector<double> sigma;
    int typeIndex;
    TransitionModels::SpatialTransType type;

    n.getParam(param_ns,paramMap);
    params.nModels = paramMap.size();

    // Assign spatial transition models, motion transition models, and labels
    for (auto it = paramMap.begin(); it != paramMap.end(); it++)
    {
        motionLabel = it->first;
        params.MotionLabels.push_back(motionLabel);

        // Get sigma vector & motion model type, then create and store it
        n.getParam(param_ns + "/" + motionLabel + "/sigma",sigma);
        n.getParam(param_ns + "/" + motionLabel + "/type",typeIndex);
        type = static_cast<TransitionModels::SpatialTransType>(typeIndex);
        TransitionModels::SpatialTransition trans(type, sigma);
        params.SpatialTransModels.push_back(trans);

        // Get discrete conditionals for motion mode transition, then store
        //params.MotionTransModels.push_back(gtsam::DiscreteConditional(A | ));
    }

    return params;
};

// Helper function to print parameters
void TransitionModels::PrintTransModelParams(TransitionModels::TransModelParams params)
{
    std::cout << "Got " << params.nModels << " spatial transition models." << std::endl << std::endl;

    for (size_t ii=0; ii<params.nModels; ++ii) {
        std::cout << "Spatial Transition Model: " << ii << std::endl;
        std::cout << "Label: " << params.MotionLabels[ii] << std::endl;
        std::cout << "Type: " << (TransitionModels::SpatialTransType)params.SpatialTransModels[ii].ModelType() << std::endl;
        std::cout << "Transition Matrix: " << std::endl;
        std::cout << params.SpatialTransModels[ii].TransModel() << std::endl;
        std::cout << "Transition Variance Vector: " << std::endl;
        std::cout << params.SpatialTransModels[ii].TransVar() << std::endl;
        std::cout << "Transition Covariance Matrix: " << std::endl;
        // gtsam::noiseModel::Diagonal tc = (*params.SpatialTransModels[ii].TransCov()); // TODO - how tho
        //tc.print;
        std::cout << "Input model: " << std::endl;
        std::cout << params.SpatialTransModels[ii].InputModel() << std::endl;
        std::cout << "Input: " << std::endl;
        std::cout << params.SpatialTransModels[ii].Input() << std::endl << std::endl;
    };
};


TransitionModels::SpatialTransition::SpatialTransition(TransitionModels::SpatialTransType type, std::vector<double> const noise_var)
    : type_(type)
{ 

    for (size_t ii =0; ii< noise_var.size(); ++ii) {
        this->transNoiseVar_(ii) = noise_var[ii];
    };

    switch(this->type_)
    {
        case TransitionModels::SpatialTransType::const_pos:
            this->transModel_ = gtsam::Matrix::Zero(6,6);
            gtsam::insertSub(this->transModel_, gtsam::Matrix::Identity(3,3),0,0); // upper left corner only is identity for constant position transition model
            break;

        case TransitionModels::SpatialTransType::const_vel:
            this->transModel_ = gtsam::Matrix::Identity(6,6);
            break;
    }

}

// Accessors
TransitionModels::SpatialTransType TransitionModels::SpatialTransition::ModelType() { return this->type_; }
gtsam::Matrix TransitionModels::SpatialTransition::TransModel() { return this->transModel_; }
gtsam::Vector6 TransitionModels::SpatialTransition::TransVar() { return this->transNoiseVar_; }
gtsam::SharedDiagonal TransitionModels::SpatialTransition::TransCov() { return this->transNoiseCov_; }
gtsam::Matrix TransitionModels::SpatialTransition::InputModel() { return this->inputModel_; }
gtsam::Vector6 TransitionModels::SpatialTransition::Input() { return this->input_; }

// Mutators
void TransitionModels::SpatialTransition::TransModel(double dt)
{
    gtsam::insertSub(this->transModel_, gtsam::Matrix::Identity(3,3)*dt,0,3);

}
    
void TransitionModels::SpatialTransition::TransCov(double dt)
{
    // std::cout << this->transNoiseVar_ << std::cout;
    this->transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(this->transNoiseVar_(0)*pow(dt,2),
                       this->transNoiseVar_(1)*pow(dt,2),
                       this->transNoiseVar_(2)*pow(dt,2),
                       this->transNoiseVar_(3)*dt,
                       this->transNoiseVar_(4)*dt,
                       this->transNoiseVar_(5)*dt));
}

void TransitionModels::SpatialTransition::UpdateTrans(double dt)
{
    switch(this->type_)
    {
        case TransitionModels::SpatialTransType::const_vel:
            this->TransModel(dt);
            this->TransCov(dt);
            break;
    }
}

#endif
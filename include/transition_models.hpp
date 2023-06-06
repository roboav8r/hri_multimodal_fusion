#ifndef TRANS_MODELS_H
#define TRANS_MODELS_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

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

TransModelParams ExtractTransModelParams(std::string, ros::NodeHandle);

class SpatialTransition
{
    public:
    // Construct from yaml input
    SpatialTransition(SpatialTransType, std::vector<double>);

    // Accessors
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
    gtsam::Vector6 transNoiseVar_;
    gtsam::SharedDiagonal transNoiseCov_;
    gtsam::Matrix transModel_;
    const gtsam::Matrix inputModel_;
    const gtsam::Vector6 input_;
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
    //std::vector<TransitionModels::MotionTransition> MotionTransModels;
    std::vector<std::string> MotionLabels;
};

// Helper function to extract transition model params from ros param path
TransitionModels::TransModelParams TransitionModels::ExtractTransModelParams(std::string param_ns, ros::NodeHandle n)
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
    }

    return params;
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
    // TODO check that this is working properly
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

//     private:
//     SpatialTransType type_;
//     gtsam::Vector6 transNoiseVar_;
//     gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
//     gtsam::Matrix transModel_ = gtsam::Matrix::Zero(6,6); // "F" matrix
//     const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
//     const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector
// }; // SpatialTransition class







// class ConstPos 
// {
//     public:
//     // Default Constructor
//     ConstPos()
//     {
//         transNoiseVar_= gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0., 0., 0.);
//         transModel_(0,0) = 1;
//         transModel_(1,1) = 1;
//         transModel_(2,2) = 1;
//     };

//     // Construct from input vector
//     ConstPos(gtsam::Vector6 noise_var) : transNoiseVar_(noise_var)
//     {
//         transModel_(0,0) = 1;
//         transModel_(1,1) = 1;
//         transModel_(2,2) = 1;
//     };

//     // Construct from yaml input
//     ConstPos(std::vector<double> const noise_var) 
//     {
//         for (int32_t ii =0; ii< noise_var.size(); ++ii) {
//             transNoiseVar_(ii) = noise_var[ii];
//         };
//     };

//     // Accessors
//     gtsam::Matrix TransModel()
//     {
//         return transModel_;
//     }

//     gtsam::Vector6 TransVar()
//     {
//         return transNoiseVar_;
//     }

//     gtsam::SharedDiagonal TransCov()
//     {
//         return transNoiseCov_;
//     }

//     gtsam::Matrix InputModel()
//     {
//         return inputModel_;
//     }

//     gtsam::Vector6 Input()
//     {
//         return input_;
//     }

//     // Mutators
//     void UpdateTrans(double dt)
//     {} // This is a placeholder since ConstPos doesn't need to update the trans

//     private:
//     gtsam::Vector6 transNoiseVar_; //= gtsam::Vector(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
//     gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
//     gtsam::Matrix transModel_ = gtsam::Matrix::Zero(6,6); // "F" matrix
//     const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
//     const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector

// }; // ConstPos class


// class ConstVel 
// {
//     public:
//     // Default Constructor
//     ConstVel()
//     {
//         transNoiseVar_= gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
//     };

//     // Construct from input vector
//     ConstVel(gtsam::Vector6 noise_var) : transNoiseVar_(noise_var) {};

//     // Construct from yaml input
//     ConstVel(std::vector<double> const noise_var) 
//     {
//         for (int32_t ii =0; ii< noise_var.size(); ++ii) {
//             transNoiseVar_(ii) = noise_var[ii];
//         };
//     };

//     // Accessors
//     gtsam::Matrix TransModel()
//     {
//         return transModel_;
//     }

//     gtsam::Vector6 TransVar()
//     {
//         return transNoiseVar_;
//     }

//     gtsam::SharedDiagonal TransCov()
//     {
//         return transNoiseCov_;
//     }

//     gtsam::Matrix InputModel()
//     {
//         return inputModel_;
//     }

//     gtsam::Vector6 Input()
//     {
//         return input_;
//     }

//     // Mutators
//     void TransModel(double dt)
//     {
//         transModel_(0,3) = dt;
//         transModel_(1,4) = dt;
//         transModel_(2,5) = dt;
//     }
    
//     void TransCov(double dt)
//     {
//         this->transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(transNoiseVar_(0)*pow(dt,2),
//                                                           transNoiseVar_(1)*pow(dt,2),
//                                                           transNoiseVar_(2)*pow(dt,2),
//                                                           transNoiseVar_(3)*dt,
//                                                           transNoiseVar_(4)*dt,
//                                                           transNoiseVar_(5)*dt));
//     }

//     void UpdateTrans(double dt)
//     {
//         TransModel(dt);
//         TransCov(dt);
//     }

//     private:
//     gtsam::Vector6 transNoiseVar_; //= gtsam::Vector(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
//     gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
//     gtsam::Matrix transModel_ = gtsam::Matrix::Identity(6,6); // "F" matrix
//     const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
//     const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector

// }; // ConstVel class

#endif
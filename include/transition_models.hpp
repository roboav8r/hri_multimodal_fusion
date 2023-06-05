#ifndef TRANS_MODELS_H
#define TRANS_MODELS_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// TODO Refactor!
// - spatial trans model struct or class
// - model = makeconstpos(noise vec)
// - model = makeconstvel(noise vec)
// void UpdateTrans(model&, dt&)

namespace TransitionModels {

class ConstPos 
{
    public:
    // Default Constructor
    ConstPos()
    {
        transNoiseVar_= gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0., 0., 0.);
        transModel_(0,0) = 1;
        transModel_(1,1) = 1;
        transModel_(2,2) = 1;
    };

    // Construct from input vector
    ConstPos(gtsam::Vector6 noise_var) : transNoiseVar_(noise_var)
    {
        transModel_(0,0) = 1;
        transModel_(1,1) = 1;
        transModel_(2,2) = 1;
    };

    // Construct from yaml input
    ConstPos(std::vector<double> const noise_var) 
    {
        for (int32_t ii =0; ii< noise_var.size(); ++ii) {
            transNoiseVar_(ii) = noise_var[ii];
        };
    };

    // Accessors
    gtsam::Matrix TransModel()
    {
        return transModel_;
    }

    gtsam::Vector6 TransVar()
    {
        return transNoiseVar_;
    }

    gtsam::SharedDiagonal TransCov()
    {
        return transNoiseCov_;
    }

    gtsam::Matrix InputModel()
    {
        return inputModel_;
    }

    gtsam::Vector6 Input()
    {
        return input_;
    }

    // Mutators
    void UpdateTrans(double dt)
    {} // This is a placeholder since ConstPos doesn't need to update the trans

    private:
    gtsam::Vector6 transNoiseVar_; //= gtsam::Vector(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
    gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
    gtsam::Matrix transModel_ = gtsam::Matrix::Zero(6,6); // "F" matrix
    const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
    const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector

};

}; // TransitionModels namespace


class ConstVel 
{
    public:
    // Default Constructor
    ConstVel()
    {
        transNoiseVar_= gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
    };

    // Construct from input vector
    ConstVel(gtsam::Vector6 noise_var) : transNoiseVar_(noise_var) {};

    // Construct from yaml input
    ConstVel(std::vector<double> const noise_var) 
    {
        for (int32_t ii =0; ii< noise_var.size(); ++ii) {
            transNoiseVar_(ii) = noise_var[ii];
        };
    };

    // Accessors
    gtsam::Matrix TransModel()
    {
        return transModel_;
    }

    gtsam::Vector6 TransVar()
    {
        return transNoiseVar_;
    }

    gtsam::SharedDiagonal TransCov()
    {
        return transNoiseCov_;
    }

    gtsam::Matrix InputModel()
    {
        return inputModel_;
    }

    gtsam::Vector6 Input()
    {
        return input_;
    }

    // Mutators
    void TransModel(double dt)
    {
        transModel_(0,3) = dt;
        transModel_(1,4) = dt;
        transModel_(2,5) = dt;
    }
    
    void TransCov(double dt)
    {
        this->transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(transNoiseVar_(0)*pow(dt,2),
                                                          transNoiseVar_(1)*pow(dt,2),
                                                          transNoiseVar_(2)*pow(dt,2),
                                                          transNoiseVar_(3)*dt,
                                                          transNoiseVar_(4)*dt,
                                                          transNoiseVar_(5)*dt));
    }

    void UpdateTrans(double dt)
    {
        TransModel(dt);
        TransCov(dt);
    }

    private:
    gtsam::Vector6 transNoiseVar_; //= gtsam::Vector(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
    gtsam::SharedDiagonal transNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(transNoiseVar_); // "Q" matrix
    gtsam::Matrix transModel_ = gtsam::Matrix::Identity(6,6); // "F" matrix
    const gtsam::Matrix inputModel_ = gtsam::Matrix::Zero(6,6); // "B" matrix
    const gtsam::Vector6 input_ = gtsam::Vector::Zero(6); // "u" vector

};

}; // TransitionModels namespace

#endif
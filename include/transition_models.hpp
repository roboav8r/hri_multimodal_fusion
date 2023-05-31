#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

class ConstVelMotion 
{
    public:
    // Constructor
    ConstVelMotion()
    {
        transNoiseVar_= gtsam::Vector6(135.688255, 98.0265414, 395.476227, 0.100000196, 0.0999837353, 0.0997463441);
    };

    ConstVelMotion(gtsam::Vector6 noise_var) : transNoiseVar_(noise_var) {};

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
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

class OakDSensor
{
    public:
    OakDSensor(gtsam::Vector3 noise_var) 
        : oakDNoiseVar_(noise_var), oakDNoiseCov_(gtsam::noiseModel::Diagonal::Sigmas(noise_var))
        {};

    // Accessors
    gtsam::Matrix MeasModel() {
        return oakDMeasModel_;
    }

    gtsam::Vector3 NoiseVar()
    {
        return oakDNoiseVar_;
    }

    gtsam::SharedDiagonal NoiseCov()
    {
        return oakDNoiseCov_;
    }

    private:
    const gtsam::Matrix oakDMeasModel_ = (gtsam::Matrix(3,6)<< 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0).finished();
    const gtsam::Vector3 oakDNoiseVar_;
    const gtsam::SharedDiagonal oakDNoiseCov_;
};
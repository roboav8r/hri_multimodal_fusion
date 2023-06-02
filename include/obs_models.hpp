#ifndef OBS_MODELS_H
#define OBS_MODELS_H


#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace Sensors {

struct SensorParams
{
    std::string name;
    std::string type;
    std::string topic;
    std::vector<double> variance;
    double pDetect;
};

class OakDSensor
{
    public:
    OakDSensor(gtsam::Vector3 noise_var) 
        : oakDNoiseVar_(noise_var), oakDNoiseCov_(gtsam::noiseModel::Diagonal::Sigmas(noise_var))
        {};
    
    // Construct from yaml input
    OakDSensor(std::vector<double> const sensor_var) 
    {
        for (int32_t ii =0; ii< sensor_var.size(); ++ii) {
            oakDNoiseVar_(ii) = sensor_var[ii];
        };
    };

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
    gtsam::Vector3 oakDNoiseVar_;
    const gtsam::SharedDiagonal oakDNoiseCov_;
};

}; //Sensors namespace

#endif
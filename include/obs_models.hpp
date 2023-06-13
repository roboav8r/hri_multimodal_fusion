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
        oakDNoiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(oakDNoiseVar_);
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
    gtsam::SharedDiagonal oakDNoiseCov_;
};

enum ClutterType
{
    FRUSTRUM_3D = 0
};

class Clutter3D
{
    public:
    // Constructors
    Clutter3D(ClutterType type, int n_objects, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
        : nObs_(n_objects), xMin_(x_min), xMax_(x_max), yMin_(y_min), yMax_(y_max), zMin_(z_min), zMax_(z_max)
        {
            // Compute volume based on geometry type
            switch(type)
            {
                case ClutterType::FRUSTRUM_3D:
                    vol_ = 1; //TODO
                    break;
                default:
                    vol_ = 1;
            }
            // Compute density
            density_ = 1/vol_;
        };

    // Accessors
    int Lambda() 
    {
        return nObs_;
    }

    double Pdf()
    {
        return density_;
    }
    
    private:
    int nObs_;
    double xMin_, xMax_, yMin_, yMax_, zMin_, zMax_, density_, vol_;
};


Clutter3D ExtractClutterParams(std::string param_ns, ros::NodeHandle n)
{
    int typeIndex, nObjects;
    ClutterType type;
    double xMin, xMax, yMin, yMax, zMin, zMax;

    n.getParam(param_ns + "type", typeIndex);
    n.getParam(param_ns + "lambda_c", nObjects);
    n.getParam(param_ns + "x_min", xMin);
    n.getParam(param_ns + "x_max", xMax);
    n.getParam(param_ns + "y_min", yMin);
    n.getParam(param_ns + "y_min", yMax);
    n.getParam(param_ns + "z_min", zMin);
    n.getParam(param_ns + "z_min", zMax);
    type = ClutterType(typeIndex);

    return Clutter3D(type, nObjects, xMin, xMax, yMin, yMax, zMin, zMax);
};



}; //Sensors namespace

#endif
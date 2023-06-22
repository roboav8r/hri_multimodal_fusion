#ifndef OBS_MODELS_H
#define OBS_MODELS_H


#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


namespace Sensors {

// Sensor types - for generating sensor observation model matrices
enum SensorType
{
    pos_3d
};


class SensorModel
{
    public:
    SensorModel(){}; // Default constructor

    // Construct from yaml input
    SensorModel(SensorType type, std::vector<double> const sensor_var)
    : type_(type)
    {
        // Construct noise covariance
        for (int32_t ii =0; ii< sensor_var.size(); ++ii) {
            noiseVar_(ii) = sensor_var[ii];
        };
        noiseCov_ = gtsam::noiseModel::Diagonal::Sigmas(noiseVar_);

        // Construct obs matrix based on type
        switch(this->type_)
        {
            case SensorType::pos_3d:
                this->measModel_ = gtsam::Matrix::Zero(3,6);
                gtsam::insertSub(this->measModel_, gtsam::Matrix::Identity(3,3),0,0); // upper left corner only is identity for constant position transition model
                std::cout << "Got meas model" << std::endl;
                std::cout << measModel_ << std::endl;
                break;
        }

    };

    // Accessors
    gtsam::Matrix MeasModel() {
        return measModel_;
    }

    gtsam::Vector3 NoiseVar()
    {
        return noiseVar_;
    }

    gtsam::SharedDiagonal NoiseCov()
    {
        return noiseCov_;
    }

    private:
    gtsam::Matrix measModel_;
    gtsam::Vector3 noiseVar_;
    gtsam::SharedDiagonal noiseCov_;
    std::vector<std::string> measLabels_;
    gtsam::Matrix labelMeasProb_; // TODO add likelihood matrix
    SensorType type_;
};


// Structure containing all observation model data
struct ObsModelParams
{
    size_t nSensors;
    std::vector<std::string> SensorNames;
    std::vector<std::string> SensorTopics;
    SensorModel SensorMdl; // TODO - adapt this for multiple sensors 
    std::vector<std::string> ClassLabels; // One per filter, should be same for all sensors
};

ObsModelParams ExtractSensorParams(std::string param_ns, ros::NodeHandle& n)
{
    ObsModelParams params;
    std::string sensorName;

    int typeIndex;
    SensorType type;
    std::string topic;
    std::vector<double> sigma;

    XmlRpc::XmlRpcValue sensorParamMap;
    n.getParam(param_ns,sensorParamMap);
    params.nSensors = sensorParamMap.size();


    // Assign observation models to params structure
    for (auto it = sensorParamMap.begin(); it != sensorParamMap.end(); it++)
    {
        sensorName = it->first;
        params.SensorNames.push_back(sensorName);

        // Get sensor type, topic, and variance
        n.getParam(param_ns + "/" + sensorName + "/type",typeIndex);
        n.getParam(param_ns + "/" + sensorName + "/topic",topic);
        n.getParam(param_ns + "/" + sensorName + "/sigma",sigma);

        type = static_cast<SensorType>(typeIndex);
        params.SensorTopics.push_back(topic);

        // TODO - generate measurement likelihood matrix, and add to SensorModel
        // TODO - generate class list

        // TODO - adapt this for multiple sensors
        SensorModel sensor(type, sigma);
        params.SensorMdl = sensor;

    }

    return params;
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
#ifndef OBS_MODELS_H
#define OBS_MODELS_H

#include <map>
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
    SensorModel(SensorType type, std::vector<double> const sensor_var, 
                std::vector<std::string> x_labels, std::vector<std::string> z_labels, 
                gtsam::Matrix label_prob, std::map<int, int> label_map)
    : type_(type), classLabels_(x_labels), measLabels_(z_labels), nX_(x_labels.size()), nZ_(z_labels.size()), labelProb_(labelProb_), labelMap_(label_map)
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
    gtsam::Matrix MeasModel() {return measModel_;}
    gtsam::Vector3 NoiseVar(){return noiseVar_;}
    gtsam::SharedDiagonal NoiseCov(){return noiseCov_;}

    private:
    // Sensor info
    SensorType type_;

    // Spatial info
    gtsam::Matrix measModel_;
    gtsam::Vector3 noiseVar_;
    gtsam::SharedDiagonal noiseCov_;

    // Semantic info
    std::vector<std::string> measLabels_;
    std::vector<std::string> classLabels_;
    size_t nX_; // Number of class labels
    size_t nZ_; // Number of measurement labels
    gtsam::Matrix labelProb_; // nX by nZ matrix of label probabilities p(z_i|x_i)
    std::map<int, int> labelMap_;
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
    std::vector<std::string> xLabels;
    std::vector<std::string> zLabels;
    std::vector<double> labelProbRow;
    gtsam::Matrix labelProb;
    std::map<int, int> labelMap;

    XmlRpc::XmlRpcValue sensorParamMap, xMap, zMap, probXml, labelXml;
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

        // Generate class labels
        n.getParam(param_ns + "/" + sensorName + "/class_labels",xMap);
        for (int ii=0; ii<xMap.size(); ii++) {xLabels.push_back(xMap[ii]);}
        
        // Generate measurement labels
        n.getParam(param_ns + "/" + sensorName + "/meas_labels",zMap);
        for (int ii=0; ii<zMap.size(); ii++) {zLabels.push_back(zMap[ii]);}

        // Generate measurement likelihood matrix, and add to SensorModel
        labelProb = gtsam::Matrix::Zero(xMap.size(), zMap.size());
        n.getParam(param_ns + "/" + sensorName + "/label_probability",probXml);
        for (int ii=0; ii<probXml.size(); ii++)
        { for (int jj=0; jj<probXml[ii].size(); jj++) {labelProb(ii,jj) = probXml[ii][jj];} } 

        // Generate sensor -> user label map
        n.getParam(param_ns + "/" + sensorName + "/label_map",labelXml);
        std::cout << labelXml.getType() << std::endl;
        for (auto it = labelXml.begin(); it != labelXml.end(); ++it)
        {
            labelMap.insert({std::stoi(it->first),std::stoi(it->second)});            
        }

        // TODO - adapt this for multiple sensor types
        SensorModel sensor(type, sigma, xLabels, zLabels, labelProb, labelMap);
        params.SensorMdl = sensor;
        params.ClassLabels = xLabels; // TODO ensure these are all the same for all sensors (xLabels are state variable)
        
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
    Clutter3D() {};

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
#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/discrete/DiscreteDistribution.h>

class ObjectState
{
    public:
    ObjectState(size_t n_models, size_t n_classes) 
        : Spatial(std::vector<gtsam::KalmanFilter::State>(n_models)), ClassLabels(std::vector<std::string>(n_classes))
        {
            Motion = gtsam::Matrix::Ones(n_models, 1);
            Motion = Motion/n_models; // Assume uniform distribution of motion probabilities
            Likelihood = gtsam::Vector::Ones(n_models);
            Likelihood = Likelihood/n_models; // Assume uniform distribution of motion probabilities

            Class = gtsam::Matrix::Ones(n_classes, 1);
            Class = Class/n_classes; // Assume uniform distribution of class             
        };

    // Spatial state
    std::vector<gtsam::KalmanFilter::State> Spatial;

    // Motion/activity state
    gtsam::Matrix Motion;
    gtsam::Vector Likelihood;

    // Class label state
    gtsam::Matrix Class;
    std::vector<std::string> ClassLabels;
};
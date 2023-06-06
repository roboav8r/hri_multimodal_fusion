#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/discrete/DiscreteDistribution.h>

class ObjectState
{
    public:
    ObjectState(size_t n_models) : Spatial(std::vector<gtsam::KalmanFilter::State>(n_models)) {};


    std::vector<gtsam::KalmanFilter::State> Spatial;
    gtsam::DiscreteDistribution Motion;
};
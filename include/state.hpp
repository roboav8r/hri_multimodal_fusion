#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/discrete/DiscreteDistribution.h>

class ObjectState
{
    public:
    ObjectState(size_t n_models) 
        : Spatial(std::vector<gtsam::KalmanFilter::State>(n_models))
        {
            MotionKey = gtsam::DiscreteKey(0, n_models);
            std::vector<double> MotionVec(n_models, (double)1/n_models);
            MotionType = gtsam::DiscreteDistribution(MotionKey, MotionVec);
        };

    // Spatial state
    std::vector<gtsam::KalmanFilter::State> Spatial;

    // Motion/activity state
    gtsam::DiscreteKey MotionKey;
    gtsam::DiscreteConditional PredictedMotion;
    gtsam::DiscreteDistribution MotionType;
};
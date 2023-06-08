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
            Motion = gtsam::DiscreteDistribution(MotionKey, MotionVec);
            Motion.print("Object motion probability prior:");
        };

    // Spatial state
    std::vector<gtsam::KalmanFilter::State> Spatial;

    // Motion/activity state
    gtsam::DiscreteKey MotionKey;
    gtsam::DiscreteDistribution Motion;
    std::vector<gtsam::DiscreteKey> PredictMotionKeys;
    gtsam::DecisionTreeFactor JointMotion;
};
#include <gtsam/linear/KalmanFilter.h>

class ObjectState
{
    public:
    gtsam::KalmanFilter::State x; // TODO replace this with spatial state?
    // TODO add categorical dist. for activity
};
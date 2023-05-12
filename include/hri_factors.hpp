#include <gtsam_unstable/dynamics/PoseRTV.h>

// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDInferenceFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactor> shared_ptr;

    OakDInferenceFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Pose3& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T.x() - mx_, T.y() - my_, T.z() - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactor(*this)));}
};


class OakDInferenceFactorRTV: public gtsam::NoiseModelFactor1<gtsam::PoseRTV> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::PoseRTV>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactorRTV> shared_ptr;

    OakDInferenceFactorRTV(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::PoseRTV>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactorRTV() override {}

    gtsam::Vector evaluateError (const gtsam::PoseRTV& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T.t().x() - mx_, T.t().y() - my_, T.t().z() - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactorRTV(*this)));}
};
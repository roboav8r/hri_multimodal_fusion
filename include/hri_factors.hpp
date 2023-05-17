#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDInferenceFactor: public gtsam::NoiseModelFactor1<gtsam::Vector3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Vector3>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactor> shared_ptr;

    OakDInferenceFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Vector3>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Vector3& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T(0) - mx_, T(1) - my_, T(2) - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactor(*this)));}
};

// class OakDInferenceFactorPose: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
//     double mx_,my_,mz_; // x,y,z measurements

// public:

//     using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

//     typedef std::shared_ptr<OakDInferenceFactorPose> shared_ptr;

//     OakDInferenceFactorPose(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
//         gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), mx_(x), my_(y), mz_(z) {}

//     ~OakDInferenceFactorPose() override {}

//     gtsam::Vector evaluateError (const gtsam::Pose3& T, gtsam::OptionalMatrixType H) const override
//     {
//         if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
//         return (gtsam::Vector(3) << T.x() - mx_, T.y() - my_, T.z() - mz_).finished();
//     }

//     gtsam::NonlinearFactor::shared_ptr clone() const override {
//         return std::static_pointer_cast<gtsam::NonlinearFactor>(
//             gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactorPose(*this)));}
// };


// class OakDInferenceFactorRTV: public gtsam::NoiseModelFactor1<gtsam::PoseRTV> {
//     double mx_,my_,mz_; // x,y,z measurements

// public:

//     using gtsam::NoiseModelFactor1<gtsam::PoseRTV>::evaluateError;

//     typedef std::shared_ptr<OakDInferenceFactorRTV> shared_ptr;

//     OakDInferenceFactorRTV(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
//         gtsam::NoiseModelFactor1<gtsam::PoseRTV>(model, j), mx_(x), my_(y), mz_(z) {}

//     ~OakDInferenceFactorRTV() override {}

//     gtsam::Vector evaluateError (const gtsam::PoseRTV& T, gtsam::OptionalMatrixType H) const override
//     {
//         if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
//         return (gtsam::Vector(3) << T.t().x() - mx_, T.t().y() - my_, T.t().z() - mz_).finished();
//     }

//     gtsam::NonlinearFactor::shared_ptr clone() const override {
//         return std::static_pointer_cast<gtsam::NonlinearFactor>(
//             gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactorRTV(*this)));}
// };


/**
 * Constraint to enforce dynamics between the velocities and poses, using
 * a prediction based on a numerical integration flag.
 *
 * NOTE: this approximation is insufficient for large timesteps, but is accurate
 * if timesteps are small.
 */
class StateTransition : public gtsam::NoiseModelFactorN<gtsam::Vector3,gtsam::Vector3> {

protected:

  double dt_;   /// time difference between frames in seconds

public:

  StateTransition(gtsam::Key key1, gtsam::Key key2, double dt, const gtsam::SharedNoiseModel& model)
  : gtsam::NoiseModelFactor2<gtsam::Vector3,gtsam::Vector3>(model, key1, key2), dt_(dt) {}

  ~StateTransition() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new StateTransition(*this))); }

  /**
   * Calculates the error for trapezoidal model given
   */
  gtsam::Vector evaluateError(const gtsam::Vector3& T1, const gtsam::Vector3& T2,
      gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override {

    if (H1) (*H1) = (gtsam::Matrix(3,3) << -1., 0., 0., 0., -1., 0., 0., 0., -1.).finished();
    if (H2) (*H2) = (gtsam::Matrix(3,3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.).finished();

    return (gtsam::Vector(3) << T2(0) - T1(0), T2(1) - T1(1), T2(2) - T1(2)).finished();

  }

};
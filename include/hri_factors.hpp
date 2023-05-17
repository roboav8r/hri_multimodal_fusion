#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDInferenceFactor: public gtsam::NoiseModelFactor1<gtsam::Vector6> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Vector6>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactor> shared_ptr;

    OakDInferenceFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Vector6>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Vector6& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(6,6) << 1, 0, 0, 0, 0, 0, 
                                             0, 1, 0, 0, 0, 0,
                                             0, 0, 1, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(6) << T(0) - mx_, 
                                    T(1) - my_, 
                                    T(2) - mz_,
                                    0,
                                    0,
                                    0).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactor(*this)));}
};


/**
 * Constraint to enforce dynamics between the velocities and poses, using
 * a prediction based on a numerical integration flag.
 *
 * NOTE: this approximation is insufficient for large timesteps, but is accurate
 * if timesteps are small.
 */
class StateTransition : public gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6> {

protected:

  double dt_;   /// time difference between frames in seconds

public:

  StateTransition(gtsam::Key key1, gtsam::Key key2, double dt, const gtsam::SharedNoiseModel& model)
  : gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6>(model, key1, key2), dt_(dt) {}

  ~StateTransition() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new StateTransition(*this))); }

  /**
   * Calculates the error for trapezoidal model given
   */
  gtsam::Vector evaluateError(const gtsam::Vector6& T1, const gtsam::Vector6& T2,
      gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override {

    if (H1) (*H1) = (gtsam::Matrix(6,6) << -1., 0., 0., 0., 0., 0., 
                                            0., -1., 0., 0., 0., 0.,
                                            0., 0., -1., 0., 0., 0.,
                                            0., 0., 0., -0.5*dt_, 0., 0.,
                                            0., 0., 0., 0., -0.5*dt_, 0.,
                                            0., 0., 0., 0., 0., -0.5*dt_).finished();
    if (H2) (*H2) = (gtsam::Matrix(6,6) << 1., 0., 0., 0., 0., 0., 
                                           0., 1., 0., 0., 0., 0.,
                                           0., 0., 1., 0., 0., 0.,
                                           0., 0., 0., -0.5*dt_, 0., 0.,
                                           0., 0., 0., 0., -0.5*dt_, 0.,
                                           0., 0., 0., 0., 0., -0.5*dt_).finished();

    return (gtsam::Vector(6) << T2(0) - (T1(0) + 0.5*dt_*(T1(3)+T2(3))),
                                T2(1) - (T1(1) + 0.5*dt_*(T1(4)+T2(4))), 
                                T2(2) - (T1(2) + 0.5*dt_*(T1(5)+T2(5))),
                                2*T2(0) - 2*T1(0) - dt_*(T1(3)+T2(3)),
                                2*T2(1) - 2*T1(1) - dt_*(T1(4)+T2(4)),
                                2*T2(2) - 2*T1(2) - dt_*(T1(5)+T2(5))).finished();

  }

};
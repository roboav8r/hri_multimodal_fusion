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

// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDCalibrationFactor: public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector3>::evaluateError;

    typedef std::shared_ptr<OakDCalibrationFactor> shared_ptr;

    OakDCalibrationFactor(gtsam::Key j, gtsam::Key k, double x, double y, double z):
        gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector3>(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1,1,1,0,0,0)), j, k), mx_(x), my_(y), mz_(z) {}

    ~OakDCalibrationFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Vector6& X, const gtsam::Vector3& Sigma, gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2 ) const override
    {
        if (H1) (*H1) = (gtsam::Matrix(6,6) << 1/Sigma(0), 0, 0, 0, 0, 0, 
                                             0, 1/Sigma(1), 0, 0, 0, 0,
                                             0, 0, 1/Sigma(2), 0, 0, 0,
                                             0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0).finished(); // Oak-D should directly measure position in camera frame
        if (H2) (*H2) = (gtsam::Matrix(6,3) << -(X(0) - mx_)/pow(Sigma(0),2), 0, 0,
                                               0, -(X(1) - my_)/pow(Sigma(1),2), 0,
                                               0, 0, -(X(2) - mz_)/pow(Sigma(2),2),
                                               0, 0, 0,
                                               0, 0, 0,
                                               0, 0, 0).finished(); 

        return (gtsam::Vector(6) << (X(0) - mx_)/Sigma(0), 
                                    (X(1) - my_)/Sigma(1), 
                                    (X(2) - mz_)/Sigma(2),
                                    0,
                                    0,
                                    0).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDCalibrationFactor(*this)));}
};


class StateTransition : public gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6> {

protected:

  double dt_;   /// time difference between frames in seconds
  gtsam::Vector6 mean_;

public:

  StateTransition(gtsam::Key key1, gtsam::Key key2, const double dt, const gtsam::Vector6& Mean, const gtsam::SharedNoiseModel& model)
  : gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6>(model, key1, key2), dt_(dt), mean_(Mean) {}

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

    return (gtsam::Vector(6) << T2(0) - (T1(0) + 0.5*dt_*(T1(3)+T2(3))) - mean_(0),
                                T2(1) - (T1(1) + 0.5*dt_*(T1(4)+T2(4))) - mean_(1), 
                                T2(2) - (T1(2) + 0.5*dt_*(T1(5)+T2(5))) - mean_(2),
                                2*T2(0) - 2*T1(0) - dt_*(T1(3)+T2(3)) - dt_*mean_(3),
                                2*T2(1) - 2*T1(1) - dt_*(T1(4)+T2(4)) - dt_*mean_(4),
                                2*T2(2) - 2*T1(2) - dt_*(T1(5)+T2(5)) - dt_*mean_(5)).finished();

  }

};

class ConstVelStateTransition : public gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6> {

protected:

  double dt_;   /// time difference between frames in seconds

public:

  ConstVelStateTransition(gtsam::Key key1, gtsam::Key key2, const double dt, const gtsam::SharedNoiseModel& model)
  : gtsam::NoiseModelFactor2<gtsam::Vector6,gtsam::Vector6>(model, key1, key2), dt_(dt) {}

  ~ConstVelStateTransition() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new ConstVelStateTransition(*this))); }

  /**
   * Calculates the error
   */
  gtsam::Vector evaluateError(const gtsam::Vector6& T1, const gtsam::Vector6& T2,
      gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override {

    if (H1) (*H1) = (gtsam::Matrix(6,6) << -1., 0., 0., -dt_, 0., 0., 
                                            0., -1., 0., 0., -dt_, 0.,
                                            0., 0., -1., 0., 0., -dt_,
                                            0., 0., 0., -1., 0., 0.,
                                            0., 0., 0., 0., -1., 0.,
                                            0., 0., 0., 0., 0., -1.).finished();
    if (H2) (*H2) = (gtsam::Matrix(6,6) << 1., 0., 0., 0., 0., 0., 
                                           0., 1., 0., 0., 0., 0.,
                                           0., 0., 1., 0., 0., 0.,
                                           0., 0., 0., 1., 0., 0.,
                                           0., 0., 0., 0., 1., 0.,
                                           0., 0., 0., 0., 0., 1.).finished();

    return (gtsam::Vector(6) << T2(0) - T1(0) - dt_*T1(3),
                                T2(1) - T1(1) - dt_*T1(4), 
                                T2(2) - T1(2) - dt_*T1(5),
                                T2(3) - T1(3),
                                T2(4) - T1(4),
                                T2(5) - T1(5)).finished();

  }

};



class StateTransitionCalibration : public gtsam::NoiseModelFactor4<gtsam::Vector6,gtsam::Vector6,gtsam::Vector6,gtsam::Vector6> {

protected:

  double dt_;   /// time difference between frames in seconds

public:

  StateTransitionCalibration(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, double dt)
  : gtsam::NoiseModelFactor4<gtsam::Vector6,gtsam::Vector6,gtsam::Vector6,gtsam::Vector6>(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(1,1,1,1,1,1)), key1, key2, key3, key4), dt_(dt) {}

  ~StateTransitionCalibration() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new StateTransitionCalibration(*this))); }

  /**
   * Calculates the error for trapezoidal model given
   */
  gtsam::Vector evaluateError(const gtsam::Vector6& X1, const gtsam::Vector6& X2, const gtsam::Vector6& Mean, const gtsam::Vector6& Var,
      gtsam::OptionalMatrixType HX1, gtsam::OptionalMatrixType HX2, gtsam::OptionalMatrixType HMean, gtsam::OptionalMatrixType HVar) const override {

    if (HX1) (*HX1) = (gtsam::Matrix(6,6) << -1./Var(0), 0., 0., 0., 0., 0., 
                                            0., -1./Var(1), 0., 0., 0., 0.,
                                            0., 0., -1./Var(2), 0., 0., 0.,
                                            0., 0., 0., -0.5*dt_/Var(3), 0., 0.,
                                            0., 0., 0., 0., -0.5*dt_/Var(4), 0.,
                                            0., 0., 0., 0., 0., -0.5*dt_/Var(5)).finished();
    if (HX2) (*HX2) = (gtsam::Matrix(6,6) << 1./Var(0), 0., 0., 0., 0., 0., 
                                           0., 1./Var(1), 0., 0., 0., 0.,
                                           0., 0., 1./Var(2), 0., 0., 0.,
                                           0., 0., 0., -0.5*dt_/Var(3), 0., 0.,
                                           0., 0., 0., 0., -0.5*dt_/Var(4), 0.,
                                           0., 0., 0., 0., 0., -0.5*dt_/Var(5)).finished();

    if (HMean) (*HMean) = (gtsam::Matrix(6,6) << -1., 0., 0., 0., 0., 0., 
                                            0., -1., 0., 0., 0., 0.,
                                            0., 0., -1., 0., 0., 0.,
                                            0., 0., 0., -1, 0., 0.,
                                            0., 0., 0., 0., -1, 0.,
                                            0., 0., 0., 0., 0., -1).finished();

    if (HVar) (*HVar) = (gtsam::Matrix(6,6) << 1., 0., 0., 0., 0., 0., 
                                           0., 1., 0., 0., 0., 0.,
                                           0., 0., 1., 0., 0., 0.,
                                           0., 0., 0., -0.5*dt_, 0., 0.,
                                           0., 0., 0., 0., -0.5*dt_, 0.,
                                           0., 0., 0., 0., 0., -0.5*dt_).finished();

    return (gtsam::Vector(6) << (X2(0) - (X1(0) + 0.5*dt_*(X1(3)+X2(3))) - Mean(0))/Var(0),
                                (X2(1) - (X1(1) + 0.5*dt_*(X1(4)+X2(4))) - Mean(1))/Var(1), 
                                (X2(2) - (X1(2) + 0.5*dt_*(X1(5)+X2(5))) - Mean(2))/Var(2),
                                (2*X2(0) - 2*X1(0) - dt_*(X1(3)+X2(3)) - dt_*Mean(3))/Var(3),
                                (2*X2(1) - 2*X1(1) - dt_*(X1(4)+X2(4)) - dt_*Mean(4))/Var(4),
                                (2*X2(2) - 2*X1(2) - dt_*(X1(5)+X2(5)) - dt_*Mean(5))/Var(5)).finished();

  }

};
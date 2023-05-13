#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

// Measurement factors
// See https://github.com/borglab/gtsam/blob/develop/examples/LocalizationExample.cpp
class OakDInferenceFactor: public gtsam::NoiseModelFactor1<gtsam::Point3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Point3>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactor> shared_ptr;

    OakDInferenceFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Point3>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactor() override {}

    gtsam::Vector evaluateError (const gtsam::Point3& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T.x() - mx_, T.y() - my_, T.z() - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactor(*this)));}
};

class OakDInferenceFactorPose: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    double mx_,my_,mz_; // x,y,z measurements

public:

    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    typedef std::shared_ptr<OakDInferenceFactorPose> shared_ptr;

    OakDInferenceFactorPose(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), mx_(x), my_(y), mz_(z) {}

    ~OakDInferenceFactorPose() override {}

    gtsam::Vector evaluateError (const gtsam::Pose3& T, gtsam::OptionalMatrixType H) const override
    {
        if (H) (*H) = (gtsam::Matrix(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // Oak-D should directly measure position in camera frame
        return (gtsam::Vector(3) << T.x() - mx_, T.y() - my_, T.z() - mz_).finished();
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new OakDInferenceFactorPose(*this)));}
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


/** controls which model to use for numerical integration to use for constraints */
typedef enum {
  TRAPEZOIDAL, // Constant acceleration
  EULER_START, // Constant velocity, using starting velocity
  EULER_END    // Constant velocity, using ending velocity
} IntegrationMode;



/**
 * Constraint to enforce dynamics between the velocities and poses, using
 * a prediction based on a numerical integration flag.
 *
 * NOTE: this approximation is insufficient for large timesteps, but is accurate
 * if timesteps are small.
 */
class StateTransition : public gtsam::NoiseModelFactorN<gtsam::PoseRTV,gtsam::PoseRTV> {
public:
  typedef gtsam::NoiseModelFactor2<gtsam::PoseRTV,gtsam::PoseRTV> Base;

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

protected:

  double dt_;   /// time difference between frames in seconds
  IntegrationMode integration_mode_;  ///< Numerical integration control

public:

  /**
   * Creates a constraint relating the given variables with fully constrained noise model
   */
  StateTransition(gtsam::Key key1, gtsam::Key key2, const IntegrationMode& mode,
      double dt, double mu = 1000)
  : Base(gtsam::noiseModel::Constrained::All(3, mu), key1, key2), dt_(dt), integration_mode_(mode) {}

  /**
     * Creates a constraint relating the given variables with fully constrained noise model
     * Uses the default Trapezoidal integrator
     */
    StateTransition(gtsam::Key key1, gtsam::Key key2, double dt, double mu = 1000)
    : Base(gtsam::noiseModel::Constrained::All(3, mu), key1, key2),
      dt_(dt), integration_mode_(TRAPEZOIDAL) {}

  /**
   * Creates a constraint relating the given variables with arbitrary noise model
   */
  StateTransition(gtsam::Key key1, gtsam::Key key2, const IntegrationMode& mode,
      double dt, const gtsam::SharedNoiseModel& model)
  : Base(model, key1, key2), dt_(dt), integration_mode_(mode) {}

  /**
   * Creates a constraint relating the given variables with arbitrary noise model
   * Uses the default Trapezoidal integrator
   */
  StateTransition(gtsam::Key key1, gtsam::Key key2, double dt, const gtsam::SharedNoiseModel& model)
  : Base(model, key1, key2), dt_(dt), integration_mode_(TRAPEZOIDAL) {}

  ~StateTransition() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new StateTransition(*this))); }

  /**
   * Calculates the error for trapezoidal model given
   */
  gtsam::Vector evaluateError(const gtsam::PoseRTV& x1, const gtsam::PoseRTV& x2,
      gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override {
    if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector,gtsam::PoseRTV,gtsam::PoseRTV>(
        std::bind(StateTransition::evaluateError_, std::placeholders::_1,
            std::placeholders::_2, dt_, integration_mode_), x1, x2, 1e-5);
    if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector,gtsam::PoseRTV,gtsam::PoseRTV>(
        std::bind(StateTransition::evaluateError_, std::placeholders::_1,
            std::placeholders::_2, dt_, integration_mode_), x1, x2, 1e-5);
    return evaluateError_(x1, x2, dt_, integration_mode_);
  }

  void print(const std::string& s = "", const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override {
    std::string a = "StateTransition: " + s;
    Base::print(a, formatter);
    switch(integration_mode_) {
    case TRAPEZOIDAL: std::cout << "Integration: Trapezoidal\n"; break;
    case EULER_START: std::cout << "Integration: Euler (start)\n"; break;
    case EULER_END: std::cout << "Integration: Euler (end)\n"; break;
    default: std::cout << "Integration: Unknown\n" << std::endl; break;
    }
    std::cout << "dt: " << dt_ << std::endl;
  }

private:
  static gtsam::Vector evaluateError_(const gtsam::PoseRTV& x1, const gtsam::PoseRTV& x2,
      double dt, const IntegrationMode& mode) {

    const gtsam::Velocity3& v1 = x1.v(), v2 = x2.v();
    const gtsam::Point3& p1 = x1.t(), p2 = x2.t();
    gtsam::Point3 hx(0,0,0);
    switch(mode) {
    case TRAPEZOIDAL: hx = p1 + gtsam::Point3((v1 + v2) * dt *0.5); break;
    case EULER_START: hx = p1 + gtsam::Point3(v1 * dt); break;
    case EULER_END  : hx = p1 + gtsam::Point3(v2 * dt); break;
    default: assert(false); break;
    }
    return p2 - hx;
  }
};

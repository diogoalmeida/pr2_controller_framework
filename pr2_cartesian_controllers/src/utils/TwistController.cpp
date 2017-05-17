#include <utils/TwistController.hpp>

namespace cartesian_controllers{

TwistController::TwistController(const Eigen::Matrix<double, 6, 1> &twist_gains) : gains_(twist_gains) {}
TwistController::~TwistController(){}

KDL::Twist TwistController::computeError(const KDL::Frame &current, const KDL::Frame &reference)
{
  KDL::Twist error;

  error = KDL::diff(current, reference);

  for (int i = 0; i < 6; i++)
  {
    error(i) = gains_[i]*error(i);
  }

  return error;
}
}

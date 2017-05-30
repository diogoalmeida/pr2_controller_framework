#include <utils/TwistController.hpp>

namespace cartesian_controllers{

TwistController::TwistController(const Eigen::Matrix<double, 6, 1> &twist_gains) : gains_(twist_gains) {}
TwistController::~TwistController(){}

KDL::Twist TwistController::computeError(const KDL::Frame &current, const KDL::Frame &reference)
{
  KDL::Twist error;
  std::vector<Eigen::Vector3d> rot_curr(3), rot_ref(3);
  std::vector<Eigen::AngleAxisd> error_rot(3);
  Eigen::Vector3d error_total = Eigen::Vector3d::Zero();

  error = KDL::diff(current, reference);
  rot_curr[0] << current.M.UnitX().data[0], current.M.UnitX().data[1], current.M.UnitX().data[2];
  rot_curr[1] << current.M.UnitY().data[0], current.M.UnitY().data[1], current.M.UnitY().data[2];
  rot_curr[2] << current.M.UnitZ().data[0], current.M.UnitZ().data[1], current.M.UnitZ().data[2];

  rot_ref[0] << reference.M.UnitX().data[0], reference.M.UnitX().data[1], reference.M.UnitX().data[2];
  rot_ref[1] << reference.M.UnitY().data[0], reference.M.UnitY().data[1], reference.M.UnitY().data[2];
  rot_ref[2] << reference.M.UnitZ().data[0], reference.M.UnitZ().data[1], reference.M.UnitZ().data[2];

  for (int i = 0; i < 3; i++)
  {
    error(i) = gains_[i]*error(i);
    error_rot[i] = getAngleAxis(rot_ref[i], rot_curr[i]);
    error_total += gains_[i + 3]*error_rot[i].angle()*error_rot[i].axis();
  }

  for (int i = 0; i < 3; i++)
  {
    error(i + 3) = error_total[i];
  }

  return error;
}

Eigen::AngleAxisd TwistController::getAngleAxis(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  double angle = 0;

  axis = v1.cross(v2);

  if (axis.norm() != 0)
  {
    axis.normalize();
    angle = acos(v1.dot(v2));
  }

  return Eigen::AngleAxisd(angle, axis);
}
}

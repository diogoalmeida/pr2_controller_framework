#include <pr2_algorithms/folding_assembly_estimator.hpp>

namespace manipulation_algorithms{

  FoldingAssemblyEstimator::FoldingAssemblyEstimator()
  {
    P_ = Eigen::Matrix3d::Identity();
  }

  FoldingAssemblyEstimator::~FoldingAssemblyEstimator(){}

  bool FoldingAssemblyEstimator::getParams(const ros::NodeHandle &n)
  {
    if(!parseMatrixData(Q_, "/folding_controller/estimator/Q", n))
    {
      return false;
    }

    if(!parseMatrixData(R_, "/folding_controller/estimator/R", n))
    {
      return false;
    }

    return true;
  }

  Eigen::Vector3d FoldingAssemblyEstimator::estimate(const Eigen::Vector3d &omega, const Eigen::Vector3d &force, const Eigen::Vector3d &torque, const double dt)
  {
    Eigen::Matrix3d A, C, K, I;

    I = Eigen::Matrix3d::Identity();
    A = computeSkewSymmetric(omega);
    C = -computeSkewSymmetric(force);

    // process model
    r_ = r_ + A*r_*dt;

    A = I + A*dt;

    P_ = A*P_ + P_*A.transpose() + R_;

    K = P_*C.transpose()*(C*P_*C.transpose() + Q_).inverse();

    // correct the process model value with the innovation (prediction error from the observation model)
    r_ = r_ + K*(torque - C*r_);

    P_ = (I - K*C)*P_;

    return r_;
  }

  void FoldingAssemblyEstimator::initialize(const Eigen::Vector3d &r)
  {
    r_ = r;
  }
}

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
    Eigen::Matrix3d A, C, K, I, P_hat, S;
    Eigen::Vector3d innov;

    I = Eigen::Matrix3d::Identity();
    A = computeSkewSymmetric(omega);
    C = -computeSkewSymmetric(force);

    // process model
    P_hat = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;
    r_ = r_ + A*r_;
    innov = torque - C*r_;
    S = C*P_hat.selfadjointView<Eigen::Upper>()*C.transpose() + Q_;
    K = P_hat.selfadjointView<Eigen::Upper>()*C.transpose()*S.llt().solve(I);
    r_ = r_ + K*innov;
    P_= (I - K*C)*P_hat.selfadjointView<Eigen::Upper>();
    // P_ = A*P_ + P_*A.transpose() + R_;

    // std::cout << std::endl << "Begin estimator" << std::endl;
    // std::cout << "A: " << A << std::endl;
    // std::cout << "K: " << K << std::endl;
    // std::cout << "P: " << P_ << std::endl;
    // std::cout << "C: " << C << std::endl;
    // std::cout << "Inovation: " << torque - C*r_ << std::endl;
    // std::cout << "End estimator" << std::endl << std::endl;

    // correct the process model value with the innovation (prediction error from the observation model)

    return r_;
  }

  void FoldingAssemblyEstimator::initialize(const Eigen::Vector3d &r)
  {
    r_ = r;
  }
}

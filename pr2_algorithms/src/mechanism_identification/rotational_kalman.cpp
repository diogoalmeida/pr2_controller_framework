#include <pr2_algorithms/mechanism_identification/rotational_kalman.hpp>

namespace manipulation_algorithms{

  RotationalEstimator::RotationalEstimator()
  {
    P_ = Eigen::Matrix3d::Identity();
  }

  RotationalEstimator::~RotationalEstimator(){}

  bool RotationalEstimator::getParams(const ros::NodeHandle &n)
  {
    if(!parseMatrixData(Q_, "/mechanism_controller/rotational_estimator/Q", n))
    {
      return false;
    }

    if(!parseMatrixData(R_, "/mechanism_controller/rotational_estimator/R", n))
    {
      return false;
    }

    return true;
  }

  Eigen::Vector3d RotationalEstimator::estimate(const Eigen::Vector3d &w_r, double dt)
  {
    Eigen::Matrix3d I, P_hat, K, S, C = Eigen::Matrix3d::Zero();
    Eigen::Vector3d innov = Eigen::Vector3d::Zero(), w_normalized;
    double w_norm;

    I = Eigen::Matrix3d::Identity();

    // process model
    w_normalized = w_r.normalized();
    P_hat = R_;
    if (w_norm > 0)
    {
      C = I - w_normalized*w_normalized.transpose();
      innov = -C*k_;
      S = C*P_hat.selfadjointView<Eigen::Upper>()*C.transpose() + Q_;

      K = P_hat.selfadjointView<Eigen::Upper>()*C.transpose()*S.llt().solve(I);
      k_ = k_ + K*innov;
      k_ = k_/k_.norm();
      P_= (I - K*C.transpose())*P_hat.selfadjointView<Eigen::Upper>();
    }

    return k_;
  }

  void RotationalEstimator::initialize(const Eigen::Vector3d &k)
  {
    k_ = k;
  }
}

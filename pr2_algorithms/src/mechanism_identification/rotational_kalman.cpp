#include <pr2_algorithms/mechanism_identification/rotational_kalman.hpp>

namespace manipulation_algorithms{

  RotationalEstimator::RotationalEstimator()
  {
    P_ = Eigen::Matrix3d::Identity();
  }

  RotationalEstimator::~RotationalEstimator(){}

  bool RotationalEstimator::getParams(const ros::NodeHandle &n)
  {
    if(!n.getParam("/mechanism_controller/rotational_estimator/q", q_))
    {
      ROS_ERROR("Missing observational noise (/mechanism_controller/rotational_estimator/q)");
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
    Eigen::Matrix3d I, P_hat;
    Eigen::Vector3d K;
    double innov, S;

    I = Eigen::Matrix3d::Identity();

    // process model
    P_hat = R_;
    innov = w_r.dot(k_) - w_r.norm();
    S = w_r.transpose()*P_hat.selfadjointView<Eigen::Upper>()*w_r + q_;

    K = P_hat.selfadjointView<Eigen::Upper>()*w_r/S;
    k_ = k_ + K*innov;
    k_ = k_/k_.norm();
    P_= (I - K*w_r.transpose())*P_hat.selfadjointView<Eigen::Upper>();

    return k_;
  }

  void RotationalEstimator::initialize(const Eigen::Vector3d &k)
  {
    k_ = k;
  }
}

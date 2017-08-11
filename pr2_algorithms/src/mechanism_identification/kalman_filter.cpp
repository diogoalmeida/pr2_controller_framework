#include <pr2_algorithms/mechanism_identification/kalman_filter.hpp>

namespace manipulation_algorithms{

  KalmanEstimator::KalmanEstimator()
  {
    P_ = Eigen::Matrix3d::Identity();
  }

  KalmanEstimator::~KalmanEstimator(){}

  bool KalmanEstimator::getParams(const ros::NodeHandle &n)
  {
    if(!parseMatrixData(Q_, "/mechanism_controller/estimator/Q", n))
    {
      return false;
    }

    if(!parseMatrixData(R_, "/mechanism_controller/estimator/R", n))
    {
      return false;
    }

    if(!n.getParam("/mechanism_controller/estimator/constant_gain", constant_gain_))
    {
      ROS_ERROR("Missing constant gain (/mechanism_controller/estimator/constant_gain)");
      return false;
    }

    return true;
  }

  Eigen::Vector3d KalmanEstimator::estimateConstant(const Eigen::Vector3d &p_e1, const Vector6d &x_dot_e1, const Eigen::Vector3d &p_e2, const Vector6d &wrench_e2, double dt)
  {
    Eigen::Matrix3d A, C, K, I, P_hat, S;
    Eigen::Vector3d innov, force, torque, v1, w1;

    I = Eigen::Matrix3d::Identity();
    v1 = x_dot_e1.block<3, 1>(0, 0);
    w1 = x_dot_e1.block<3, 1>(3, 0);
    force = wrench_e2.block<3,1>(0, 0);
    torque = wrench_e2.block<3,1>(3, 0);

    A = computeSkewSymmetric(w1);
    C = -computeSkewSymmetric(force);

    // process model
    pc_ = pc_ + (A*(pc_ - p_e1) + v1)*dt;
    innov = torque + C*p_e2 - C*pc_;

    K = constant_gain_*I;
    pc_ = pc_ + K*innov;

    return pc_;
  }

  Eigen::Vector3d KalmanEstimator::estimate(const Eigen::Vector3d &p_e1, const Vector6d &x_dot_e1, const Eigen::Vector3d &p_e2, const Vector6d &wrench_e2, double dt)
  {
    Eigen::Matrix3d A, C, K, I, P_hat, S;
    Eigen::Vector3d innov, force, torque, v1, w1;

    I = Eigen::Matrix3d::Identity();
    v1 = x_dot_e1.block<3, 1>(0, 0);
    w1 = x_dot_e1.block<3, 1>(3, 0);
    force = wrench_e2.block<3,1>(0, 0);
    torque = wrench_e2.block<3,1>(3, 0);

    A = computeSkewSymmetric(w1);
    C = -computeSkewSymmetric(force);

    // process model
    P_hat = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;
    pc_ = pc_ + (A*(pc_ - p_e1) + v1)*dt;
    innov = torque + C*p_e2 - C*pc_;
    S = C*P_hat.selfadjointView<Eigen::Upper>()*C.transpose() + Q_;

    K = P_hat.selfadjointView<Eigen::Upper>()*C.transpose()*S.llt().solve(I);
    pc_ = pc_ + K*innov;
    P_= (I - K*C)*P_hat.selfadjointView<Eigen::Upper>();
    // P_ = A*P_ + P_*A.transpose() + R_;

    // std::cout << std::endl << "Begin estimator" << std::endl;
    // std::cout << "A: " << A << std::endl;
    // std::cout << "K: " << K << std::endl;
    // std::cout << "P: " << P_ << std::endl;
    // std::cout << "C: " << C << std::endl;
    // std::cout << "Inovation: " << torque - C*pc_ << std::endl;
    // std::cout << "End estimator" << std::endl << std::endl;

    // correct the process model value with the innovation (prediction error from the observation model)

    return pc_;
  }

  void KalmanEstimator::initialize(const Eigen::Vector3d &pc)
  {
    pc_ = pc;
  }
}

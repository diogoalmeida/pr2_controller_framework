#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(){}
  ECTSController::~ECTSController(){}

  Vector14d ECTSController::control(const Matrix67d &J_1, const Matrix67d &J_2, const Vector3d &r_1, const Vector3d &r_2, const Vector7d &q_dot_1, const Vector7d &q_dot_2, const Vector12d &error)
  {
    MatrixECTS J = MatrixECTS::Zero();
    Matrix12d C, W = Matrix12d::Identity();
    Eigen::Matrix<double, 14, 12> damped_inverse, sigma = Eigen::Matrix<double, 14, 12>::Zero();
    Vector14d q_dot, epsilon;
    int sing_values_num;

    J.block<6, 7>(0, 0) = J_1;
    J.block<6, 7>(6, 7) = J_2;
    C.block<6, 6>(0, 0) = alpha_*Matrix6d::Identity();
    C.block<6, 6>(0, 6) = (1 - alpha_)*Matrix6d::Identity();
    C.block<6, 6>(6, 0) = -beta_*Matrix6d::Identity();
    C.block<6, 6>(6, 6) = Matrix6d::Identity();
    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1);
    W.block<3, 3>(3, 9) = -computeSkewSymmetric(r_2);
    J = C*W*J; // ECTS Jacobian
    q_dot.block<7, 1>(0, 0) = q_dot_1;
    q_dot.block<7, 1>(7, 0) = q_dot_2;
    epsilon = nullSpaceTask(J, Vector12d::Identity());
    
    damped_inverse = J.transpose()*(J*J.transpose() + damping_*Matrix12d::Identity()).inverse();

    return damped_inverse*K_*error + epsilon - J.colPivHouseholderQr().solve(epsilon);
  }

  Vector14d ECTSController::nullSpaceTask(const MatrixECTS &J, const Vector12d &u)
  {
    return Vector14d::Zero();
  }

  bool ECTSController::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/mechanism_controller/ects_controller/alpha", alpha_))
    {
      ROS_ERROR("Missing alpha gain (/mechanism_controller/ects_controller/alpha)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/ects_controller/beta", beta_))
    {
      ROS_ERROR("Missing beta value (/mechanism_controller/ects_controller/beta)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/ects_controller/inverse_damping", damping_))
    {
      ROS_ERROR("Missing damping value (/mechanism_controller/ects_controller/inverse_damping)");
      return false;
    }

    if (beta_ != 0 && beta_ != 1)
    {
      ROS_ERROR("Beta can only be 0 or 1 (got %d)", beta_);
      return false;
    }

    // TODO: Initialize K_
    K_ = Matrix12d::Identity();
    return true;
  }
}

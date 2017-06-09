#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(){}
  ECTSController::~ECTSController(){}

  Vector14d ECTSController::control(const Matrix67d &J_1, const Matrix67d &J_2, const Vector3d &r_1, const Vector3d &r_2, const Vector7d &q_dot_1, const Vector7d &q_dot_2, const Vector12d &error)
  {
    MatrixECTS J = MatrixECTS::Zero();
    Matrix12d C, W = Matrix12d::Identity();
    Vector14d q_dot, epsilon;

    J.block<6, 7>(0, 0) = J_1;
    J.block<6, 7>(6, 7) = J_2;
    C.block<6, 6>(0, 0) = alpha_*Matrix6d::Identity();
    C.block<6, 6>(0, 6) = (1 - alpha_)*Matrix6d::Identity();
    C.block<6, 6>(6, 0) = -beta_*Matrix6d::Identity();
    C.block<6, 6>(6, 6) = Matrix6d::Identity();
    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1);
    W.block<3, 3>(3, 9) = -computeSkewSymmetric(r_2);
    q_dot.block<7, 1>(0, 0) = q_dot_1;
    q_dot.block<7, 1>(7, 0) = q_dot_2;
    epsilon = nullSpaceTask(J, Vector12d::Identity());

    return J.transpose()*(J*J.transpose() + damping_*Matrix12d::Identity()).inverse()*K_*error + (Eigen::Matrix<double, 14, 14>::Identity() - J.transpose()*(J*J.transpose()).inverse()*J)*epsilon;
  }

  Vector14d ECTSController::nullSpaceTask(const MatrixECTS &J, const Vector12d &u)
  {
    return Vector14d::Zero();
  }

  bool ECTSController::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/ects_controller/alpha", alpha_))
    {
      ROS_ERROR("Missing alpha gain (/ects_controller/alpha)");
      return false;
    }

    if (!n.getParam("/ects_controller/beta", beta_))
    {
      ROS_ERROR("Missing beta value (/ects_controller/beta)");
      return false;
    }

    if (beta_ != 0 && beta_ != 1)
    {
      ROS_ERROR("Beta can only be 0 or 1 (got %d)", beta_);
      return false;
    }

    // TODO: Initialize K_

    return true;
  }
}
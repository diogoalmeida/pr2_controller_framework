#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2)
  {
    jac_dot_solver_1_.reset(new KDL::ChainJntToJacDotSolver(chain_1));
    jac_dot_solver_2_.reset(new KDL::ChainJntToJacDotSolver(chain_2));
  }

  ECTSController::~ECTSController(){}

  Vector14d ECTSController::control(const Matrix67d &J_1, const Matrix67d &J_2, const Vector3d &r_1, const Vector3d &r_2, const Vector7d &q_dot_1, const Vector7d &q_dot_2, const Vector6d &twist_a, const Vector6d &twist_r)
  {
    MatrixECTSr J = MatrixECTSr::Zero();
    Matrix6d damped_inverse;
    Matrix12d C, W = Matrix12d::Identity();
    Vector14d q_dot, epsilon;

    // J.block<6, 7>(0, 0) = J_1;
    // J.block<6, 7>(6, 7) = J_2;
    // C.block<6, 6>(0, 0) = alpha_*Matrix6d::Identity();
    // C.block<6, 6>(0, 6) = (1 - alpha_)*Matrix6d::Identity();
    // C.block<6, 6>(6, 0) = -beta_*Matrix6d::Identity();
    // C.block<6, 6>(6, 6) = Matrix6d::Identity();
    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1);
    W.block<3, 3>(3, 9) = -computeSkewSymmetric(r_2);
    J.block<6,7>(0,0) = -beta_*W.block<6,6>(0,0)*J_1;
    J.block<6,7>(0,7) = W.block<6,6>(6,6)*J_2; // ECTS Jacobian

    q_dot.block<7, 1>(0, 0) = q_dot_1;
    q_dot.block<7, 1>(7, 0) = q_dot_2;
    // epsilon = nullSpaceTask(J, Vector12d::Identity());

    damped_inverse = (J*J.transpose() + damping_*Matrix6d::Identity());

    return J.transpose()*damped_inverse.ldlt().solve(twist_r) + epsilon - J.householderQr().solve(J*epsilon);
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

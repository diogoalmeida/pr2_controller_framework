#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2)
  {
    jac_solver_1_.reset(new KDL::ChainJntToJacSolver(chain_1));
    jac_solver_2_.reset(new KDL::ChainJntToJacSolver(chain_2));
  }

  ECTSController::~ECTSController(){}

  Vector14d ECTSController::control(const Vector3d &r_1, const Vector3d &r_2, const KDL::JntArray &q1, const KDL::JntArray &q2, const Vector6d &twist_a, const Vector6d &twist_r)
  {
    MatrixECTSr J = MatrixECTSr::Zero();
    Matrix6d damped_inverse;
    Vector14d q_dot, epsilon = Vector14d::Zero();

    q1_ = q1;
    q2_ = q2;
    r_1_ = r_1;
    r_2_ = r_2;

    J = computeRelativeJacobian(q1_, q2_);

    epsilon = computeNullSpaceTask();

    damped_inverse = (J*J.transpose() + damping_*Matrix6d::Identity());

    return J.transpose()*damped_inverse.ldlt().solve(twist_r) + epsilon - J.householderQr().solve(J*epsilon);
  }

  MatrixECTSr ECTSController::computeRelativeJacobian(const KDL::JntArray &q1, const KDL::JntArray &q2)
  {
    Matrix12d C, W = Matrix12d::Identity();
    MatrixECTSr J;
    KDL::Jacobian J_1_kdl, J_2_kdl;

    jac_solver_1_->JntToJac(q1, J_1_kdl);
    jac_solver_2_->JntToJac(q2, J_2_kdl);

    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1_);
    W.block<3, 3>(3, 9) = -computeSkewSymmetric(r_2_);
    J.block<6,7>(0,0) = -beta_*W.block<6,6>(0,0)*J_1_kdl.data;
    J.block<6,7>(0,7) = W.block<6,6>(6,6)*J_2_kdl.data;

    return J;
  }

  double ECTSController::computeTransmissionRatio(const MatrixECTSr &J, const Vector6d &u)
  {
    return 1/sqrt(u.transpose()*(J*J.transpose()).inverse()*u);
  }

  double ECTSController::computeTaskCompatibility(const MatrixECTSr &J)
  {
    double cm = 0, ratio;
    for (unsigned long i = 0; i < u_list_.size(); i++)
    {
      ratio = computeTransmissionRatio(J, u_list_[i]);
      cm += ratio*ratio;
    }

    return cm;
  }

  Vector14d ECTSController::computeNullSpaceTask()
  {
    Vector14d task = Vector14d::Zero();
    KDL::JntArray q_plus, q_minus;
    MatrixECTSr J_plus, J_minus;
    double epsilon = 0.001;
    double cm_plus, cm_minus;

    for (int i = 0; i < 7; i++)
    {
      q_plus = q1_;
      q_minus = q1_;
      q_plus(i) += epsilon;
      q_minus(i) -= epsilon;

      J_plus = computeRelativeJacobian(q_plus, q2_);
      J_minus = computeRelativeJacobian(q_minus, q2_);
      cm_plus = computeTaskCompatibility(J_plus);
      cm_minus = computeTaskCompatibility(J_minus);
      task(i) = (cm_plus - cm_minus)/(2*epsilon);
    }

    for (int i = 0; i < 7; i++)
    {
      q_plus = q2_;
      q_minus = q2_;
      q_plus(i) += epsilon;
      q_minus(i) -= epsilon;

      J_plus = computeRelativeJacobian(q1_, q_plus);
      J_minus = computeRelativeJacobian(q1_, q_minus);
      cm_plus = computeTaskCompatibility(J_plus);
      cm_minus = computeTaskCompatibility(J_minus);
      task(i + 7) = (cm_plus - cm_minus)/(2*epsilon);
    }

    return task;
  }

  void ECTSController::addOptimizationDirection(const Vector6d &u)
  {
    u_list_.push_back(u);
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

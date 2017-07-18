#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2)
  {
    jac_solver_1_.reset(new KDL::ChainJntToJacSolver(chain_1));
    jac_solver_2_.reset(new KDL::ChainJntToJacSolver(chain_2));
    current_cm_ = 0;
    km_ = 0;
    epsilon_ = Vector14d::Zero();
    optimization_thread_ = boost::thread(boost::bind(&ECTSController::optimizationTaskLoop, this));
    q1_.resize(chain_1.getNrOfJoints());
    q2_.resize(chain_2.getNrOfJoints());
    r_1_ = Vector3d::Zero();
    r_2_ = Vector3d::Zero();
  }

  ECTSController::~ECTSController()
  {
    if (optimization_thread_.joinable())
    {
      optimization_thread_.interrupt();
      optimization_thread_.join();
    }
  }
  
  void ECTSController::optimizationTaskLoop()
  {
    double feedback_hz = 500;
    MatrixECTS J = MatrixECTS::Zero();
    
    try
    {
      while(ros::ok())
      {
        epsilon_ = computeNullSpaceTask();
        J = computeECTSJacobian(q1_, q2_);
        current_cm_ = computeTaskCompatibility(J);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz));
      }  
    }
    catch(const boost::thread_interrupted &)
    {
      return;
    }
  }

  Vector14d ECTSController::control(const Vector3d &r_1, const Vector3d &r_2, const KDL::JntArray &q1, const KDL::JntArray &q2, const Vector6d &twist_a, const Vector6d &twist_r)
  {
    MatrixECTS J = MatrixECTS::Zero();
    Matrix12d damped_inverse;
    Vector12d total_twist;
    Vector14d q_dot, epsilon = Vector14d::Zero();

    q1_ = q1;
    q2_ = q2;
    r_1_ = r_1;
    r_2_ = r_2;
    total_twist.block<6,1>(0,0) = twist_a;
    total_twist.block<6,1>(6,0) = twist_r;
    
    J = computeECTSJacobian(q1_, q2_);

    damped_inverse = (J*J.transpose() + damping_*Matrix12d::Identity());
    
    epsilon = epsilon_;
    return J.transpose()*damped_inverse.ldlt().solve(total_twist) + epsilon  - J.transpose()*(J*J.transpose()).householderQr().solve(J*epsilon);
  }

  void ECTSController::setNullspaceGain(double km)
  {
    km_ = km;
  }

  void ECTSController::setAlpha(double alpha)
  {
    alpha_ = alpha;
  }

  double ECTSController::getTaskCompatibility()
  {
    return current_cm_;
  }

  double ECTSController::getAlpha()
  {
    return alpha_;
  }

  MatrixECTS ECTSController::computeECTSJacobian(const KDL::JntArray &q1, const KDL::JntArray &q2)
  {
    Matrix12d C = Matrix12d::Zero(), W = Matrix12d::Identity();
    MatrixECTS J_e, J = MatrixECTS::Zero();
    KDL::Jacobian J_1_kdl(7), J_2_kdl(7);

    jac_solver_1_->JntToJac(q1, J_1_kdl);
    jac_solver_2_->JntToJac(q2, J_2_kdl);

    C.block<6,6>(0,0) = alpha_*Matrix6d::Identity();
    C.block<6,6>(0,6) = (1 - alpha_)*Matrix6d::Identity();
    C.block<6,6>(6,0) = -beta_*Matrix6d::Identity();
    C.block<6,6>(6,6) = Matrix6d::Identity();

    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1_);
    W.block<3, 3>(6, 9) = -computeSkewSymmetric(r_2_);

    J.block<6,7>(0,0) = J_1_kdl.data;
    J.block<6,7>(6,7) = J_2_kdl.data;

    J_e = C*W*J;

    return J_e;
  }
  
  MatrixECTS ECTSController::computeECTSJacobianGiven1(const KDL::JntArray &q2, const KDL::Jacobian &J_1_kdl)
  {
    Matrix12d C = Matrix12d::Zero(), W = Matrix12d::Identity();
    MatrixECTS J_e, J = MatrixECTS::Zero();
    KDL::Jacobian J_2_kdl(7);

    jac_solver_2_->JntToJac(q2, J_2_kdl);

    C.block<6,6>(0,0) = alpha_*Matrix6d::Identity();
    C.block<6,6>(0,6) = (1 - alpha_)*Matrix6d::Identity();
    C.block<6,6>(6,0) = -beta_*Matrix6d::Identity();
    C.block<6,6>(6,6) = Matrix6d::Identity();

    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1_);
    W.block<3, 3>(6, 9) = -computeSkewSymmetric(r_2_);

    J.block<6,7>(0,0) = J_1_kdl.data;
    J.block<6,7>(6,7) = J_2_kdl.data;

    J_e = C*W*J;

    return J_e;
  }
  
  MatrixECTS ECTSController::computeECTSJacobianGiven2(const KDL::JntArray &q1, const KDL::Jacobian &J_2_kdl)
  {
    Matrix12d C = Matrix12d::Zero(), W = Matrix12d::Identity();
    MatrixECTS J_e, J = MatrixECTS::Zero();
    KDL::Jacobian J_1_kdl(7);

    jac_solver_1_->JntToJac(q1, J_1_kdl);

    C.block<6,6>(0,0) = alpha_*Matrix6d::Identity();
    C.block<6,6>(0,6) = (1 - alpha_)*Matrix6d::Identity();
    C.block<6,6>(6,0) = -beta_*Matrix6d::Identity();
    C.block<6,6>(6,6) = Matrix6d::Identity();

    W.block<3, 3>(0, 3) = -computeSkewSymmetric(r_1_);
    W.block<3, 3>(6, 9) = -computeSkewSymmetric(r_2_);

    J.block<6,7>(0,0) = J_1_kdl.data;
    J.block<6,7>(6,7) = J_2_kdl.data;

    J_e = C*W*J;

    return J_e;
  }

  double ECTSController::computeTransmissionRatio(const MatrixECTS &J, const Vector12d &u)
  {
    Matrix12d dJJ = (J*J.transpose()).inverse();
    double quad = u.transpose()*dJJ*u;

    return 1/sqrt(quad);
  }

  double ECTSController::computeTaskCompatibility(const MatrixECTS &J)
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
    MatrixECTS J_plus, J_minus;
    double epsilon = 0.001;
    double cm_plus, cm_minus;
    KDL::Jacobian J_1(7), J_2(7);
    jac_solver_1_->JntToJac(q1_, J_1);
    jac_solver_2_->JntToJac(q2_, J_2);
    
    for (int i = 0; i < 7; i++)
    {
      q_plus = q1_;
      q_minus = q1_;
      q_plus(i) += epsilon;
      q_minus(i) -= epsilon;

      J_plus = computeECTSJacobianGiven2(q_plus, J_2);
      J_minus = computeECTSJacobianGiven2(q_minus, J_2);
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
      
      J_plus = computeECTSJacobianGiven1(q_plus, J_1);
      J_minus = computeECTSJacobianGiven1(q_minus, J_1);
      cm_plus = computeTaskCompatibility(J_plus);
      cm_minus = computeTaskCompatibility(J_minus);
      task(i + 7) = (cm_plus - cm_minus)/(2*epsilon);
    }

    return km_*task;
  }

  void ECTSController::addOptimizationDirection(const Vector12d &u)
  {
    u_list_.push_back(u);
  }

  void ECTSController::clearOptimizationDirections()
  {
    u_list_.clear();
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

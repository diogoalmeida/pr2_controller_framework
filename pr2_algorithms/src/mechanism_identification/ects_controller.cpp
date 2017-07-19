#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>

namespace manipulation_algorithms{

  ECTSController::ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2)
  {
    jac_solver_1_.reset(new KDL::ChainJntToJacSolver(chain_1));
    jac_solver_2_.reset(new KDL::ChainJntToJacSolver(chain_2));
    current_cm_ = 0;
    km_ = 0;
    optimization_hz_ = 1;
    max_nullspace_velocities_ = 0;
    gradient_delta_ = 0.0001;
    epsilon_ = Vector14d::Zero();
    optimization_thread_ = boost::thread(boost::bind(&ECTSController::optimizationTaskLoop, this));
    q1_.resize(chain_1.getNrOfJoints());
    KDL::SetToZero(q1_);
    q2_.resize(chain_2.getNrOfJoints());
    KDL::SetToZero(q2_);
    r_1_ = Vector3d::Zero();
    r_2_ = Vector3d::Zero();
    dyncfg_cb_ = boost::bind(&ECTSController::reconfigureCallback, this, _1, _2);
    dyncfg_server_.setCallback(dyncfg_cb_);
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
    MatrixECTS J = MatrixECTS::Zero();

    try
    {
      while(ros::ok())
      {
        {
          // boost::lock_guard<boost::mutex> guard(optimization_mutex_);
          epsilon_ = computeNullSpaceTask();
          // std::cout << "joint values from optimization loop: ";
          // for (int i = 0; i < 7; i++)
          // {
          //   std::cout << "q1: " << q1_(i) << " q2: " << q2_(i) << std::endl;
          // }
          // std::cout << "epsilon from optimization loop: " << epsilon_ << std::endl;
          J = computeECTSJacobian(q1_, q2_);
          current_cm_ = computeTaskCompatibility(J);
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/optimization_hz_));
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
    Matrix14d I = Matrix14d::Identity();
    Matrix12d damped_inverse;
    Vector12d total_twist;
    Vector14d q_dot, epsilon = Vector14d::Zero(), proj;

    q1_ = q1;
    q2_ = q2;
    r_1_ = r_1;
    r_2_ = r_2;
    total_twist.block<6,1>(0,0) = twist_a;
    total_twist.block<6,1>(6,0) = twist_r;

    J = computeECTSJacobian(q1_, q2_);

    damped_inverse = (J*J.transpose() + damping_*Matrix12d::Identity());

    // std::cout << "alpha: " << alpha_ << " beta: " << beta_ << " km_: " << km_ << " max_nullspace_velocities: " << max_nullspace_velocities_ << " damping: " << damping_ << " gradient_delta: " << gradient_delta_ << std::endl;

    {
      // boost::lock_guard<boost::mutex> guard(optimization_mutex_);
      epsilon = epsilon_;
      // std::cout << "Epsilon: " << std::endl << epsilon << std::endl << std::endl;
    }

    proj = (I  - J.transpose()*(J*J.transpose() + damping_*Matrix12d::Identity()).inverse()*J)*epsilon;

    for (int i = 0; i < 14; i++)
    {
      if (proj[i] > max_nullspace_velocities_)
      {
        proj = Vector14d::Zero();
        break;
      }
    }

    // std::cout << "Proj: " << std::endl << proj.transpose() << std::endl << std::endl;
    return J.transpose()*damped_inverse.ldlt().solve(total_twist) + proj;
  }

  void ECTSController::reconfigureCallback(const pr2_algorithms::ectsConfig &config, uint32_t level)
  {
    setAlpha(config.ects_alpha);
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
    // boost::lock_guard<boost::mutex> guard(optimization_mutex_);
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

  double ECTSController::computeTransmissionRatio(const MatrixECTS &J, const Vector12d &u)
  {
    Matrix12d dJJ = (J*J.transpose() + damping_*Matrix12d::Identity()).inverse();
    double quad = u.transpose()*dJJ*u;

    // std::cout << "quad: " << quad << std::endl;

    return 1/sqrt(quad);
  }

  double ECTSController::computeTaskCompatibility(const MatrixECTS &J)
  {
    double cm = 0, ratio;
    for (unsigned long i = 0; i < u_list_.size(); i++)
    {
      ratio = computeTransmissionRatio(J, u_list_[i]);
      // std::cout << "ratio " << ratio << std::endl;
      cm += ratio*ratio;
    }

    if (std::isnan(cm))
    {
      ROS_WARN("TASK COMPATIBILITY IS NAN");
      cm = 0;
    }

    // std::cout << "returning " << cm << std::endl;

    return cm;
  }

  Vector14d ECTSController::computeNullSpaceTask()
  {
    Vector14d task = Vector14d::Zero();
    KDL::JntArray q_plus, q_minus;
    MatrixECTS J_plus, J_minus;
    double cm_plus, cm_minus;

    // std::cout << "gradient_delta: " << gradient_delta_ << std::endl;

    for (int i = 0; i < 7; i++)
    {
      q_plus = q1_;
      q_minus = q1_;
      q_plus(i) += gradient_delta_;
      q_minus(i) -= gradient_delta_;

      J_plus = computeECTSJacobian(q_plus, q2_);
      J_minus = computeECTSJacobian(q_minus, q2_);
      cm_plus = computeTaskCompatibility(J_plus);
      cm_minus = computeTaskCompatibility(J_minus);
      // std::cout << "cm_plus: " << cm_plus << " cm_minus: " << cm_minus << std::endl;
      task(i) = (cm_plus - cm_minus)/(2*gradient_delta_);
    }

    for (int i = 0; i < 7; i++)
    {
      q_plus = q2_;
      q_minus = q2_;
      q_plus(i) += gradient_delta_;
      q_minus(i) -= gradient_delta_;

      J_plus = computeECTSJacobian(q_plus, q1_);
      J_minus = computeECTSJacobian(q_minus, q1_);
      cm_plus = computeTaskCompatibility(J_plus);
      cm_minus = computeTaskCompatibility(J_minus);
      // std::cout << "cm_plus: " << cm_plus << " cm_minus: " << cm_minus << std::endl;
      task(i + 7) = (cm_plus - cm_minus)/(2*gradient_delta_);
    }

    // std::cout << "task: " << task.transpose() << " km*task: " << km_*task.transpose() << std::endl;

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

    if (!n.getParam("/mechanism_controller/ects_controller/optimization_rate", optimization_hz_))
    {
      ROS_ERROR("Missing optimization rate (/mechanism_controller/ects_controller/optimization_rate)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/ects_controller/gradient_delta", gradient_delta_))
    {
      ROS_ERROR("Missing the numerical gradient delta (/mechanism_controller/ects_controller/gradient_delta)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/ects_controller/max_nullspace_velocities", max_nullspace_velocities_))
    {
      ROS_ERROR("Missing value for maximum nullspace joint velocities(/mechanism_controller/ects_controller/max_nullspace_velocities)");
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

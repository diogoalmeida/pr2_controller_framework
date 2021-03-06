#include <pr2_algorithms/mechanism_identification/adaptive_velocity_controller.hpp>

namespace manipulation_algorithms{

  AdaptiveController::AdaptiveController()
  {
    int_force_ = Eigen::Vector3d::Zero();
    int_torque_ = Eigen::Vector3d::Zero();
    v_f_ = Eigen::Vector3d::Zero();
    w_f_ = Eigen::Vector3d::Zero();
    normal_bias_ = 0.0;
    t_ << 1, 0, 0;
    r_ << 0, 0, 1;
  }

  AdaptiveController::~AdaptiveController(){}

  Vector6d AdaptiveController::control(const Vector6d &wrench, const Eigen::Vector3d &virtual_stick, double v_d, double w_d, double dt)
  {
    Eigen::Vector3d normal, torque_d;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Vector6d ref_twist;

    normal = t_.cross(r_);
    torque_d = Eigen::Vector3d::Zero();
    stick_ = virtual_stick;

    force_error_ = wrench.block<3,1>(0,0) - f_d_*normal;
    // force_error_ = wrench.block<3,1>(0,0) - f_d_*wrench.block<3,1>(0,0).normalized();
    torque_error_ = (wrench.block<3,1>(3,0) - normal_bias_*normal); 
    // torque_error_ = (I - normal*normal.transpose())*torque_error_;

    if (torque_error_.norm() < torque_slack_)
    {
      torque_error_ = Eigen::Vector3d::Zero();
    }
    // std::cout << "torque_error_ norm: " << torque_error_.norm() << std::endl;
    // torque_error_ = wrench.block<3,1>(3,0) - torque_d_*wrench.block<3,1>(3,0).normalized();

    int_force_ = computeIntegralTerm(int_force_, t_, force_error_, dt);
    v_f_ = alpha_force_*force_error_ + beta_force_*int_force_;
    ref_twist.block<3,1>(0,0) = v_d*t_ - (I - t_*t_.transpose())*v_f_;
    t_ = t_ - alpha_adapt_t_*v_d*(I - t_*t_.transpose())*v_f_*dt;
    t_ = t_/t_.norm();
    // t_ = t_ - alpha_adapt_t_*1*(I - t_*t_.transpose())*v_f_*dt;

    int_torque_ = computeIntegralTerm(int_torque_, r_, torque_error_, dt);
    w_f_ = alpha_torque_*(I - r_*r_.transpose())*torque_error_ + beta_torque_*int_torque_;
    ref_twist.block<3,1>(3,0) = w_d*r_ - w_f_; // The ects framework will compensate the virtual sticks
    r_ = r_ - alpha_adapt_r_*w_d*w_f_*dt;
    r_ = r_/r_.norm();

    return ref_twist;
  }

  Eigen::Vector3d AdaptiveController::computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt)
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    return prev + (I - v*v.transpose())*error*dt;
  }
  
  void AdaptiveController::getErrors(Eigen::Vector3d &force_e, Eigen::Vector3d &torque_e, Eigen::Vector3d &desired_force, Eigen::Vector3d &desired_torque)
  {
    Eigen::Vector3d normal = t_.cross(r_);
    force_e = force_error_;
    torque_e = torque_error_;
    desired_force = f_d_*normal;
    desired_torque = stick_.cross(desired_force);
  }
  
  void AdaptiveController::initEstimates(const Eigen::Vector3d &t, const Eigen::Vector3d &r)
  {
    t_ = t;
    r_ = r;
  }

  void AdaptiveController::setReferenceForce(double f_d)
  {
    f_d_ = f_d;
  }

  void AdaptiveController::getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r)
  {
    t = t_;
    r = r_;
  }

  void AdaptiveController::getForceControlValues(Eigen::Vector3d &v_f, Eigen::Vector3d &w_f)
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    v_f = (I - t_*t_.transpose())*v_f_;
    w_f = w_f_;
  }

  bool AdaptiveController::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/mechanism_controller/adaptive_estimator/alpha_force", alpha_force_))
    {
      ROS_ERROR("Missing force gain (/adaptive_estimator/alpha_force)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/beta_force", beta_force_))
    {
      ROS_ERROR("Missing beta force value (/adaptive_estimator/beta_force)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/alpha_adapt_t", alpha_adapt_t_))
    {
      ROS_ERROR("Missing translational dof adaptation value (/adaptive_estimator/alpha_adapt_t)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/alpha_torque", alpha_torque_))
    {
      ROS_ERROR("Missing torque gain (/adaptive_estimator/alpha_torque)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/beta_torque", beta_torque_))
    {
      ROS_ERROR("Missing beta torque value (/adaptive_estimator/beta_torque)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/alpha_adapt_r", alpha_adapt_r_))
    {
      ROS_ERROR("Missing translational dof adaptation value (/adaptive_estimator/alpha_adapt_r)");
      return false;
    }

    if (!n.getParam("/mechanism_controller/adaptive_estimator/torque_slack", torque_slack_))
    {
      ROS_ERROR("Missing torque slack value (/adaptive_estimator/torque_slack)");
      return false;
    }
    
    if (!n.getParam("/mechanism_controller/adaptive_estimator/torque_bias", normal_bias_))
    {
      ROS_ERROR("Missing torque bias value (/adaptive_estimator/torque_bias)");
      return false;
    }

    return true;
  }
}

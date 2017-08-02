#include <pr2_algorithms/mechanism_identification/adaptive_velocity_controller.hpp>

namespace manipulation_algorithms{

  AdaptiveController::AdaptiveController()
  {
    int_force_ = Eigen::Vector3d::Zero();
    int_torque_ = Eigen::Vector3d::Zero();
    t_ << 1, 0, 0;
    r_ << 0, 0, 1;
  }

  AdaptiveController::~AdaptiveController(){}

  Vector6d AdaptiveController::control(const Vector6d &wrench, double dt)
  {
    Eigen::Vector3d force_error, torque_error, v_f, w_f;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Vector6d ref_twist;
    double v_d, w_d;

    time_ += dt;

    force_error = f_d_*wrench.block<3,1>(0,0).normalized() - wrench.block<3,1>(0,0);
    torque_error = torque_d_*wrench.block<3,1>(3,0).normalized() - wrench.block<3,1>(3,0);

    int_force_ = computeIntegralTerm(int_force_, t_, force_error, dt);
    v_f = alpha_force_*force_error + beta_force_*int_force_;
    v_d = v_d_amp_*sin(2*M_PI*v_freq*time_);
    ref_twist.block<3,1>(0,0) = v_d*t_ - (I - t_*t_.transpose())*v_f;
    t_ = t_ + -alpha_adapt_t_*v_d*(I - t_*t_.transpose())*v_f*dt;

    int_torque_ = computeIntegralTerm(int_torque_, r_, torque_error, dt);
    w_f = alpha_torque_*torque_error + beta_torque_*int_torque_;
    w_d = w_d_amp_*sin(2*M_PI*w_freq*time_);
    ref_twist.block<3,1>(3,0) = w_d*r_ - (I - r_*r_.transpose())*w_f;
    r_ = r_ - alpha_adapt_r_*w_d*(I - r_*r_.transpose())*w_f*dt;

    return ref_twist;
  }

  Eigen::Vector3d AdaptiveController::computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt)
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    return prev + (I - v*v.transpose())*error*dt;
  }

  void AdaptiveController::initEstimates(const Eigen::Vector3d &t, const Eigen::Vector3d &r)
  {
    t_ = t;
    r_ = r;
  }

  void AdaptiveController::getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r)
  {
    t = t_;
    r = r_;
  }

  bool AdaptiveController::getParams(const ros::NodeHandle &n)
  {
    // if (!n.getParam("/ects_controller/alpha", alpha_))
    // {
    //   ROS_ERROR("Missing alpha gain (/ects_controller/alpha)");
    //   return false;
    // }
    //
    // if (!n.getParam("/ects_controller/beta", beta_))
    // {
    //   ROS_ERROR("Missing beta value (/ects_controller/beta)");
    //   return false;
    // }

    return true;
  }
}

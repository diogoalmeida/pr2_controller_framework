#ifndef __ADAPTIVE_CONTROL_ALGORITHM__
#define __ADAPTIVE_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
typedef Eigen::Matrix<double, 6, 1> Vector6d;
  /**
    Class that implements the adaptive control algorithm that enables the
    mechanism identification.
  **/
  class AdaptiveController : public AlgorithmBase
  {
  public:
    AdaptiveController();
    ~AdaptiveController();

    virtual bool getParams(const ros::NodeHandle &n);

    /**
      Computes the relative twist reference for the mechanism and updates the
      estimates.

      @param wrench The required wrench for the estimation. This can be at the end-effector, as the force will match the contact point and the angular velocity has the same restrictions as the contact's.
      @param dt The elapsed time between calls.
      @return The control twist for the relative velocity between parts.
    **/
    Vector6d control(const Vector6d &wrench, double dt);

    /**
      Initialize the adaptive controller estimates.

      @param t Initial translational DOF estimate.
      @param r Initial rotational DOF estimate.
    **/
    void initEstimates(const Eigen::Vector3d &t, const Eigen::Vector3d &r);

    /**
      Return the current estimates.
      @param t Current translational DOF estimate.
      @param r Current rotational DOF estimate.
    **/
    void getEstimates(Eigen::Vector3d &t, Eigen::Vector3d &r);

  private:
    double alpha_force_, beta_force_, alpha_torque_, beta_torque_, f_d_, torque_d_, v_d_amp_, w_d_amp_, time_, v_freq, w_freq, alpha_adapt_t_, alpha_adapt_r_;
    Eigen::Vector3d t_, r_, int_force_, int_torque_;

    /**
      Compute the integral term in the wrench feedback component of the adaptive controller.

      @param prev The previous value of the integral term.
      @param v The projection vector.
      @param error The force/torque error.
      @param dt The elapsed time.
      @return The updated integral value.
    **/
    Eigen::Vector3d computeIntegralTerm(const Eigen::Vector3d &prev, const Eigen::Vector3d &v, const Eigen::Vector3d &error, double dt);
  };
}
#endif

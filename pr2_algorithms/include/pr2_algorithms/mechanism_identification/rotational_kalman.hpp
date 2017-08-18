#ifndef __MECHANISM_ROTATIONAL_KALMAN__
#define __MECHANISM_ROTATIONAL_KALMAN__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
  /**
    Class that estimates a rotational DOF given measurements of the angular velocity along it.
  **/
  class RotationalEstimator : public AlgorithmBase
  {
  public:
    RotationalEstimator();
    ~RotationalEstimator();

    /**
      Estimate the rotational dof from measurements of relative angular velocity,
      in the same frame..

      @param w_r The measured rotational velocity.
      @param dt The elapsed time between estimate steps.
      @output The rotational dof estimate.
    **/
    Eigen::Vector3d estimate(const Eigen::Vector3d &w_r, double dt);

    /**
      Initialize the estimator

      @param k The initial dof estimate.
    **/
    void initialize(const Eigen::Vector3d &k);
    virtual bool getParams(const ros::NodeHandle &n);

  private:
    Eigen::Vector3d k_;
    Eigen::Matrix3d P_, R_, Q_;
  };
}
#endif

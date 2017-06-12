#ifndef __MECHANISM_KALMAN_ALGORITHM__
#define __MECHANISM_KALMAN_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
typedef Eigen::Matrix<double, 6, 1> Vector6d;

  /**
    Class that implements the joint position estimator used
    in the mechanism identification problem.
  **/
  class KalmanEstimator : public AlgorithmBase
  {
  public:
    KalmanEstimator();
    ~KalmanEstimator();

    /**
      Estimate the joint location from the measured reaction forces.
      Neglects inertial effects and friction.

      @param p_e1 The position of the rod piece end-effector.
      @param x_dot_e1 The rod piece end-effector twist.
      @param wrench_e1 The measured wrench of the rod piece end-effector.
      @param dt The elapsed time between estimate steps.
      @output The joint position estimate pc.
    **/
    Eigen::Vector3d estimate(const Eigen::Vector3d &p_e1, const Vector6d &x_dot_e1, const Vector6d &wrench_e1, double dt);

    /**
      Initialize the estimator

      @param pc The initial joint position estimate.
    **/
    void initialize(const Eigen::Vector3d &pc);

    virtual bool getParams(const ros::NodeHandle &n);

  private:
    Eigen::Vector3d pc_;
    Eigen::Matrix3d P_, Q_, R_;
  };
}
#endif
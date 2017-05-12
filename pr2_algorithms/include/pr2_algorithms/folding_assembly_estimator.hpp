#ifndef __FOLDING_ESTIMATION_ALGORITHM__
#define __FOLDING_ESTIMATION_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{

  /**
    Class that implements the contact point estimator
    used in the folding assembly problem.
  **/
  class FoldingAssemblyEstimator : public AlgorithmBase
  {
  public:
    FoldingAssemblyEstimator();
    ~FoldingAssemblyEstimator();

    /**
      Computes an estimate of the free vector that represents a rigidly grasped object.

      Makes use of the process model \f$\dot{\mathbf{r}} = \mathbf{S}(\boldsymbol{\omega_1}}) \mathbf{r}\f$
      and of the observation model \f$\boldsymbol{\tau} = -\mathbf{S}(\mathbf{f})\mathbf{r}\f$.

      @param omega The end-effector angular velocity.
      @param force The measured contact force.
      @param torque The measured torque at the end-effector wrist.
      @param dt Elapsed time between calls (in seconds).
      @return A free vector representing the estimate of the vector that connects the wrench measurement point to the contact point.
    **/
    Eigen::Vector3d estimate(const Eigen::Vector3d &omega, const Eigen::Vector3d &force, const Eigen::Vector3d &torque, const double dt);

    /**
      Initialize the estimator

      @param r The initial virtual stick estimate.
    **/
    void initialize(const Eigen::Vector3d &r);

    virtual bool getParams(const ros::NodeHandle &n);

  private:
    Eigen::Vector3d r_;
    Eigen::Matrix3d P_, Q_, R_;
  };
}
#endif

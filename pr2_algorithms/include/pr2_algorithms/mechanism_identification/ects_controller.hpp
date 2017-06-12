#ifndef __ECTS_CONTROL_ALGORITHM__
#define __ECTS_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 7> Matrix67d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 14> MatrixECTS;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;

  /**
    Class that implements an extended cooperative task space (ECTS) control algorithm.

    Hardcoded dimensions to fit two 7 DOF manipulators.
  **/
  class ECTSController : public AlgorithmBase
  {
  public:
    ECTSController();
    ~ECTSController();

    virtual bool getParams(const ros::NodeHandle &n);

    /**
      Computes the ECTS reference joint velocities for the two manipulators.

      @param J_i The manipulators' jacobians.
      @param r_i The virtual sticks connecting the manipulators end-effectos to the object reference points.
      @param q_dot_i The manipulators' joint velocities.
      @return The 14 dimensional joint velocities vector.
    **/
    Vector14d control(const Matrix67d &J_1, const Matrix67d &J_2, const Vector3d &r_1, const Vector3d &r_2, const Vector7d &q_dot_1, const Vector7d &q_dot_2, const Vector12d &error);
  private:
    double alpha_, damping_;
    Matrix12d K_;
    int beta_;

    /**
      Computes the secundary objective that will be projected unto the nullspace
      of the primary task.

      @param J The ECTS jacobian.
      @param u The desired axis for the optimal velocity transfer ration.
      @return The secundary objective velocity reference.
    **/
    Vector14d nullSpaceTask(const MatrixECTS &J, const Vector12d &u);
  };
}
#endif

#ifndef __ECTS_CONTROL_ALGORITHM__
#define __ECTS_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

using namespace Eigen;
namespace manipulation_algorithms{

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
    Matrix<double, 14, 1> control(const Matrix<double, 6, 7> &J_1, const Matrix<double, 6, 7> &J_2, const Vector3d &r_1, const Vector3d &r_2, const Matrix<double, 7, 1> &q_dot_1, const Matrix<double, 7, 1> &q_dot_2, const Matrix<double, 12, 1> &error);
  private:
    double alpha_, damping_, K_;
    int beta_;

    Matrix<double, 14, 1> nullSpaceTask(const Matrix<double, 12, 14> &J, const Matrix<double, 12, 1> &u);
  };
}
#endif

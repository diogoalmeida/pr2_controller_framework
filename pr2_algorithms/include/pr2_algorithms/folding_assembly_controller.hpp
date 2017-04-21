#ifndef __FOLDING_CONTROL_ALGORITHM__
#define __FOLDING_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{

  /**
    Class that implements the (master-slave) folding assembly control algorithm.
  **/
  class FoldingAssemblyController : public AlgorithmBase
  {
  public:
    FoldingAssemblyController();
    ~FoldingAssemblyController();

    void control(const Eigen::Vector3d &p_d, const double theta_d, const double f_d, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, const Eigen::Vector3d &r, const Eigen::Vector3d &p_1, const Eigen::Vector3d &f_c, Eigen::Vector3d &v_out, Eigen::Vector3d &w_out, const double d_t);
    virtual bool getParams(const ros::NodeHandle &n);
  protected:
    double k_v_, k_omega_, k_force_;
  };
}
#endif

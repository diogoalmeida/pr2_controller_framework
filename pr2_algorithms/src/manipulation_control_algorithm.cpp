#include <pr2_algorithms/manipulation_control_algorithm.hpp>

namespace manipulation_algorithms{

  ManipulationAlgorithm::ManipulationAlgorithm()
  {
    Gamma_ << Eigen::Matrix3d::Zero();
    k_s_ = 1;
  }

  ManipulationAlgorithm::~ManipulationAlgorithm()
  {
    // Bye!
  }

  bool ManipulationAlgorithm::getParams(const ros::NodeHandle &n)
  {
    double k_1, k_2, k_3;
    if (!n.getParam("/manipulation_controller/gains/k_1", k_1))
    {
      ROS_ERROR("Missing k_1 (/manipulation_controller/gains/k_1)");
      return false;
    }

    if (!n.getParam("/manipulation_controller/gains/k_2", k_2))
    {
      ROS_ERROR("Missing k_2 (/manipulation_controller/gains/k_2)");
      return false;
    }

    if (!n.getParam("/manipulation_controller/gains/k_3", k_3))
    {
      ROS_ERROR("Missing k_3 (/manipulation_controller/gains/k_3)");
      return false;
    }

    Gamma_ << k_1, 0  , 0  ,
              0  , k_2, 0  ,
              0  , 0  , k_3;

    if (!n.getParam("/manipulation_controller/spring_constant", k_s_))
    {
      ROS_ERROR("Missing spring_constant (/manipulation_controller/spring_constant)");
      return false;
    }

    return true;
  }

  Eigen::Vector3d ManipulationAlgorithm::compute(const Eigen::Vector3d &x_d, const Eigen::Vector3d &x_c, const Eigen::Vector3d &x_e)
  {
    Eigen::Matrix3d inv_G;
    Eigen::Vector3d e;

    inv_G = computeInvG(x_e[0], x_c[0], x_c[1]);
    e = x_d - x_c;

    if (e.norm() > 0.001)
    {
      return inv_G*Gamma_*e;
    }
    else
    {
      ROS_WARN("Success");
      return Eigen::Vector3d::Zero();
    }
  }

  Eigen::Matrix3d ManipulationAlgorithm::computeInvG(const double x_e, const double x_c, const double theta_c)
  {
    Eigen::Matrix3d inv;
    double d_x;

    d_x = x_e - x_c;

    if (std::abs(1/(k_s_*cos(theta_c))) < std::numeric_limits<double>::epsilon()) // prevent 0/0, defined as 0
    {
      ROS_WARN("Manipulation control algorithms numeric limit");
      inv << 1, 0               , 0,
             0, d_x             , 0,
             0, -1              , 0;
    }
    else
    {
      inv << 1, -d_x*tan(theta_c), 0                     ,
             0, d_x             , 0                      ,
             0, -1              , -d_x/(k_s_*cos(theta_c));
    }

    return inv;
  }
}
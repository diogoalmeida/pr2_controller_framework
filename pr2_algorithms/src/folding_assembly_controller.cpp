#include <pr2_algorithms/folding_assembly_controller.hpp>

namespace manipulation_algorithms{

  FoldingAssemblyController::FoldingAssemblyController(){}
  FoldingAssemblyController::~FoldingAssemblyController(){}

  void FoldingAssemblyController::control(const Eigen::Vector3d &p_d, const double theta_d, const double f_d, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, const Eigen::Vector3d &r, const Eigen::Vector3d &p_1, const Eigen::Vector3d &f_c, Eigen::Vector3d &v_out, Eigen::Vector3d &w_out, const double d_t)
  {
    double theta;
    Eigen::Vector3d rot_axis;

    theta = std::acos(r.dot(surface_tangent)/r.norm());
    rot_axis = computeSkewSymmetric(surface_normal)*surface_tangent;

    w_out = k_omega_*(theta_d - theta)*rot_axis;

    v_out = k_v_*(p_d - (r + p_1)).dot(surface_tangent)*surface_tangent + k_force_*(f_d - f_c.dot(surface_normal))*surface_normal - computeSkewSymmetric(w_out)*r;
  }

  bool FoldingAssemblyController::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/folding_controller/k_omega", k_omega_))
    {
      ROS_ERROR("Missing omega gain (/folding_controller/k_omega)");
      return false;
    }

    if (!n.getParam("/folding_controller/k_v", k_v_))
    {
      ROS_ERROR("Missing velocity gain (/folding_controller/k_v)");
      return false;
    }

    if (!n.getParam("/folding_controller/k_force", k_force_))
    {
      ROS_ERROR("Missing force gain (/folding_controller/k_force)");
      return false;
    }

    return true;
  }
}

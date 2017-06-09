#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <pr2_algorithms/dexterous_manipulation/manipulation_ekf.hpp>
#include <pr2_algorithms/dexterous_manipulation/manipulation_control_algorithm.hpp>
#include <utils/TwistController.hpp>
#include <tf/transform_broadcaster.h>
#include <limits>

namespace cartesian_controllers{

/**
  Implements the dexterous manipulation controller for a grasp with torsional compliance.
**/
class ManipulationController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                                                pr2_cartesian_controllers::ManipulationControllerResult>
{
private:
  /**
    Computes the skew-symmetric matrix of the provided vector.

    @param v The vector that will be used to build the skew-symmetric matrix.
    @return The computed skew-symmetric matrix.
  **/
  Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);

  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

private:
  double wait_for_tf_time_;
  bool has_initial_, estimate_length_;
  bool finished_acquiring_goal_;
  double estimated_length_, estimated_orientation_, hardcoded_length_, k_s_, theta_o_, surface_frame_vertical_offset_, surface_frame_horizontal_offset_;
  double init_x_offset_, init_theta_offset_, x_o_; // to initialize the ekf estimator
  Eigen::Affine3d surface_frame_, grasp_point_pose_, end_effector_pose_;
  Eigen::Vector3d estimated_r_, x_d_, x_e_;
  Eigen::VectorXd x_hat_;
  KDL::Frame initial_pose_, end_effector_to_grasp_point_;
  std::vector<double> comp_gains_;
  std::string grasp_point_frame_name_;
  int arm_index_;

  // estimator
  manipulation_algorithms::ManipulationEKF ekf_estimator_;
  // controller
  manipulation_algorithms::ManipulationAlgorithm controller_;

  // Debug parameters
  bool debug_twist_, use_debug_eef_to_grasp_, surface_rotation_axis_;
  Eigen::Vector3d debug_eef_to_grasp_eig_;
  double debug_x_, debug_y_, debug_rot_;

  ros::Publisher target_pub_, current_pub_, eef_to_grasp_pub_, ground_truth_pub_;
  tf::TransformBroadcaster broadcaster_;

  boost::shared_ptr<TwistController> twist_controller_;
public:
  ManipulationController();
  virtual ~ManipulationController();
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

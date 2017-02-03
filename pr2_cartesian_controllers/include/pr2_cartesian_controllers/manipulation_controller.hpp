#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <pr2_algorithms/manipulation_ekf.hpp>
#include <pr2_algorithms/manipulation_control_algorithm.hpp>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <limits>

namespace cartesian_controllers{

class ManipulationController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                                                pr2_cartesian_controllers::ManipulationControllerResult>
{
private:
  double wait_for_tf_time_;
  bool has_initial_, estimate_length_;
  bool finished_acquiring_goal_;
  double estimated_length_, estimated_orientation_, hardcoded_length_, k_s_, theta_o_;
  double init_x_offset_, init_theta_offset_; // to initialize the ekf estimator
  Eigen::Affine3d surface_frame_, grasp_point_pose_, end_effector_pose_;
  Eigen::Vector3d estimated_r_, x_hat_, x_d_, x_e_;
  geometry_msgs::Vector3Stamped rot_gains_;
  KDL::Frame initial_pose_, end_effector_to_grasp_point_;
  std::string grasp_point_frame_name_;

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

  Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);

  // For markers
  void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker);

  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

public:
  ManipulationController();

  virtual ~ManipulationController();
  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

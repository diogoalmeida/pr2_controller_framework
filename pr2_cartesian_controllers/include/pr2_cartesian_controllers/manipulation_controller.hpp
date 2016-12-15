#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace cartesian_controllers{

class ManipulationController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                                                pr2_cartesian_controllers::ManipulationControllerResult>
{
private:
  // Actionlib
  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

  // Controller values
  double k_spring_, estimated_length_, estimated_orientation_, hardcoded_length_, initial_angle_offset_;
  Eigen::Affine3d surface_frame_, goal_pose_, grasp_point_pose_, end_effector_pose_;
  Eigen::Matrix3d control_gains_;
  Eigen::Vector3d estimated_r_;
  geometry_msgs::Vector3Stamped rot_gains_;
  KDL::Frame initial_pose_, end_effector_to_grasp_point_;
  bool has_initial_, estimate_length_;
  void estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt);
  Eigen::Matrix3d computeInvG(double length, double angle);
  Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);
  std::string grasp_point_frame_name_;

  // Debug parameters
  bool debug_twist_, use_debug_eef_to_grasp_, surface_rotation_axis_;
  Eigen::Vector3d debug_eef_to_grasp_eig_;
  double debug_x_, debug_y_, debug_rot_;

  // For markers
  void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker);
  ros::Publisher target_pub_, current_pub_, eef_to_grasp_pub_;
  tf::TransformBroadcaster broadcaster_;

public:
  ManipulationController() : ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                pr2_cartesian_controllers::ManipulationControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    target_pub_ = nh_.advertise<visualization_msgs::Marker>("manipulation_controller_target", 1);
    current_pub_ = nh_.advertise<visualization_msgs::Marker>("manipulation_controller_estimated", 1);
    eef_to_grasp_pub_ = nh_.advertise<visualization_msgs::Marker>("eef_to_grasp_pose", 1);
    feedback_thread_ = boost::thread(boost::bind(&ManipulationController::publishFeedback, this));
  }
  virtual ~ManipulationController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
    delete action_server_;
  }

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

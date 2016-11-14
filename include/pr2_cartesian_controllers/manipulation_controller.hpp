#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
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
  double k_spring_, estimated_length_, estimated_orientation_;
  Eigen::Affine3d surface_frame_, goal_pose_, end_effector_pose_;
  Eigen::Matrix3d control_gains_;
  Eigen::Vector3d estimated_r_;
  void estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt);

public:
  ManipulationController() : ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                pr2_cartesian_controllers::ManipulationControllerResult>(){};

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

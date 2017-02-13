#ifndef __APPROACH_CONTROLLER__
#define __APPROACH_CONTROLLER__

#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

namespace cartesian_controllers{

/**
  Implements a controller responsible for a guarded approach movement.
**/
class ApproachController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::GuardedApproachAction,
                                                                            pr2_cartesian_controllers::GuardedApproachFeedback,
                                                                            pr2_cartesian_controllers::GuardedApproachResult>
{
private:
  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

private:
  // Controller values
  double initial_force_, force_threshold_;
  KDL::Twist velocity_reference_;
  KDL::Frame initial_pose_;
  std::vector<double> rot_gains_;
  bool has_initial_, is_contact_;
  ros::Duration contact_detection_time_;
  ros::Time initial_contact_;

public:
  ApproachController();
  virtual ~ApproachController();

  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

#ifndef __MOVE_CONTROLLER__
#define __MOVE_CONTROLLER__

#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

namespace cartesian_controllers{

class MoveController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                                                        pr2_cartesian_controllers::MoveFeedback,
                                                                        pr2_cartesian_controllers::MoveResult>
{
private:
  // Actionlib
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

  // Controller values
  KDL::Frame pose_reference_;

public:
  MoveController() : ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                        pr2_cartesian_controllers::MoveFeedback,
                                        pr2_cartesian_controllers::MoveResult>(){};


  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

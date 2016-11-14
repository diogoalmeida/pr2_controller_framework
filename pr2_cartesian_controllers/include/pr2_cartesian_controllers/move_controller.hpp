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
  void publishFeedback();

  // Controller values
  KDL::Frame pose_reference_;

  // ROS
  ros::Publisher target_pub_, current_pub_;

public:
  MoveController() : ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                        pr2_cartesian_controllers::MoveFeedback,
                                        pr2_cartesian_controllers::MoveResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    startActionlib();
    target_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_target", 1);
    current_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_current", 1);
    boost::thread(boost::bind(&MoveController::publishFeedback, this));
  }


  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

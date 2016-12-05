#ifndef __APPROACH_CONTROLLER__
#define __APPROACH_CONTROLLER__

#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

namespace cartesian_controllers{

class ApproachController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::GuardedApproachAction,
                                                                            pr2_cartesian_controllers::GuardedApproachFeedback,
                                                                            pr2_cartesian_controllers::GuardedApproachResult>
{
private:
    // Actionlib
  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

  // Controller values
  double force_threshold_;
  KDL::Twist velocity_reference_;

public:
  ApproachController() : ControllerTemplate<pr2_cartesian_controllers::GuardedApproachAction,
                                            pr2_cartesian_controllers::GuardedApproachFeedback,
                                            pr2_cartesian_controllers::GuardedApproachResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    startActionlib();
    feedback_thread_ = boost::thread(boost::bind(&ApproachController::publishFeedback, this));
  };
  virtual ~ApproachController()
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

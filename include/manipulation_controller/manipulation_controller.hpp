#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <manipulation_controller/ManipulationControllerAction.h>
#include <boost/thread.hpp>


namespace manipulation{

class ManipulationController
{
private:
  sensor_msgs::JointState robot_state;
  std::vector<std::string> joint_names_;

  manipulation_controller::ManipulationControllerFeedback feedback_;
  double feedback_hz_;

  void publishFeedback();

public:
  ManipulationController();
};
}

#endif

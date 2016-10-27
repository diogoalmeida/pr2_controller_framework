#ifndef __MANIPULATION_CLIENT__
#define __MANIPULATION_CLIENT__

#include <actionlib/client/simple_action_client.h>
#include <manipulation_controller/ManipulationControllerAction.h>

namespace manipulation{

  class ManipulationClient
  {
  public:
    ManipulationClient(std::string action_name) : action_client_(action_name, true)
    {
      action_name_ = action_name;
      runExperiment();
    }

  private:
    // actionlib
    actionlib::SimpleActionClient<manipulation_controller::ManipulationControllerAction> action_client_;
    std::string action_name_;
    double server_timeout_;

    // Vision feedback
    double vision_timeout_;
    bool waitForTablePose(ros::Duration max_time);

    // Experimental setup
    void runExperiment();
  };
  }
#endif

#ifndef __EXCLUSIVE_CONTROLLER_RUNNER__
#define __EXCLUSIVE_CONTROLLER_RUNNER__

#include <ros/ros.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <utils/extra.hpp>

namespace pr2_cartesian_clients{
  /*
    Makes sure that only one controller is running in the PR2 at any given time.
  */
  class ExclusiveControllerRunner
  {
  public:
    ExclusiveControllerRunner();
    ~ExclusiveControllerRunner();

    bool runController(std::string controller_name);

  private:
    ros::NodeHandle nh_;

    ros::ServiceClient load_controllers_client_;
    ros::ServiceClient unload_controllers_client_;
    ros::ServiceClient switch_controllers_client_;
    ros::ServiceClient list_controllers_client_;

    bool loadController(std::string controller_name);
    bool startController(std::string controller_name);
    bool controllerIsRunning(std::string controller_name);
    bool controllerIsLoaded(std::string controller_name);
  };
}
#endif

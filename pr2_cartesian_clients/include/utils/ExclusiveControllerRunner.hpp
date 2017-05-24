#ifndef __EXCLUSIVE_CONTROLLER_RUNNER__
#define __EXCLUSIVE_CONTROLLER_RUNNER__

#include <ros/ros.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/ListControllerTypes.h>
#include <utils/extra.hpp>

namespace pr2_cartesian_clients{
  /**
    Makes sure that only one controller is running in the PR2 at any given time.
  **/
  class ExclusiveControllerRunner
  {
  public:
    ExclusiveControllerRunner();
    ~ExclusiveControllerRunner();

    /**
      Makes sure the given controller runs in the PR2, stopping any other running
      controller.

      @param controller_name The realtime loop controller name.
      @return True if the controller starts running, false otherwise.
    **/
    bool runController(const std::string &controller_name);

    /**
      Stops a controller, ignoring the exception list.

      @param controller_name The realtime loop controller name.
      @return True if the controller stops, false otherwise.
    **/
    bool stopController(const std::string &controller_name);

    /**
      Unloads all controllers in the pr2.

      @return
    **/
    bool unloadAll();

    /**
      Adds a controller name to the exception list. These controllers will never be
      stopped or unloaded automatically and need to be explicitly chosen to be so.

      @param controller_name The name of the controller in the realtime loop to be
      excluded from automatic unloading and stopping.
      @return
    **/
    bool addException(const std::string &controller_name);

  private:
    ros::NodeHandle nh_;

    ros::ServiceClient load_controllers_client_;
    ros::ServiceClient unload_controllers_client_;
    ros::ServiceClient switch_controllers_client_;
    ros::ServiceClient list_controllers_client_;
    ros::ServiceClient list_controller_types_client_;

    std::vector<std::string> exception_list_;

    /**
      Loads the given controller in the PR2.

      @param controller_name
      @return
    **/
    bool loadController(const std::string &controller_name);

    /**
      Unloads the given controller from the PR2

      @param controller_name
      @return
    **/
    bool unloadController(const std::string &controller_name);

    /**
      Starts the given controller in the PR2.

      @param controller_name
      @return
    **/
    bool startController(const std::string &controller_name);

    /**
      Checks if a controller is running in the PR2 realtime loop.

      @param controller_name The realtime loop controller name.
      @return True if the controller is running, false otherwise.
    **/
    bool controllerIsRunning(const std::string &controller_name);

    /**
      Checks if a controller is loaded in the PR2.

      @param controller name The realtime loop controller name.
      @return True if the controller is loaded, false otherwise.
    **/
    bool controllerIsLoaded(const std::string &controller_name);
  };

  /**
    Checks if an item is in the vector
  **/
  bool isInVector(const std::string &item, const std::vector<std::string> &v);
}
#endif

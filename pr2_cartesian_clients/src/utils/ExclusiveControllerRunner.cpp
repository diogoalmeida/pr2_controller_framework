#include <utils/ExclusiveControllerRunner.hpp>

namespace pr2_cartesian_clients {
  ExclusiveControllerRunner::ExclusiveControllerRunner()
  {
    nh_ = ros::NodeHandle("~");

    load_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::LoadController>("/pr2_controller_manager/load_controller");
    unload_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::UnloadController>("/pr2_controller_manager/unload_controller");
    switch_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");
    list_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::ListControllers>("/pr2_controller_manager/list_controllers");
  }

  ExclusiveControllerRunner::~ExclusiveControllerRunner() {}

  /*
    Makes sure the given controller runs in the PR2, stopping any other running
    controller. Returns true if the controller starts running, false otherwise.
  */
  bool ExclusiveControllerRunner::runController(std::string controller_name)
  {
    if (!controllerIsLoaded(controller_name))
    {
      if (!loadController(controller_name))
      {
        ROS_ERROR("Tried to run a controller (%s) that is not available for loading!", controller_name.c_str());
        return false;
      }
    }

    if (!controllerIsRunning(controller_name))
    {
      if (!startController(controller_name))
      {
        ROS_ERROR("Controller %s is loaded, but failed to start!", controller_name.c_str());
        return false;
      }
    }

    return true;
  }

  /*
    Returns true if the given controller name is loaded in the PR2.
  */
  bool ExclusiveControllerRunner::controllerIsLoaded(std::string controller_name)
  {
    pr2_mechanism_msgs::ListControllers list_srv;

    if (!list_controllers_client_.call(list_srv))
    {
      ROS_ERROR("Error calling the list controllers server!");
      return false;
    }

    if (stringInVector(controller_name, list_srv.response.controllers))
    {
      return true;
    }

    return false;
  }

  /*
    Returns true if the controller is running in the PR2 realtime loop.
  */
  bool ExclusiveControllerRunner::controllerIsRunning(std::string controller_name)
  {
    pr2_mechanism_msgs::ListControllers list_srv;

    if (!list_controllers_client_.call(list_srv))
    {
      ROS_ERROR("Error calling the list controllers server!");
      return false;
    }

    for (int i = 0; i < list_srv.response.controllers.size(); i++)
    {
      if (list_srv.response.controllers[i] == controller_name)
      {
        if (list_srv.response.state[i] == "running")
        {
          return true;
        }
      }
    }

    return false;
  }

  /*
    Starts the given controller in the PR2.
  */
  bool ExclusiveControllerRunner::startController(std::string controller_name)
  {
    pr2_mechanism_msgs::ListControllers list_srv;
    pr2_mechanism_msgs::SwitchController switch_srv;

    if (!list_controllers_client_.call(list_srv))
    {
      ROS_ERROR("Error calling the list controllers server!");
      return false;
    }

    for (int i = 0; i < list_srv.response.controllers.size(); i++)
    {
      if (list_srv.response.controllers[i] != controller_name)
      {
        if (list_srv.response.state[i] == "running")
        {
          switch_srv.request.stop_controllers.push_back(list_srv.response.controllers[i]);
        }
      }
      else
      {
        if (list_srv.response.state[i] == "running")
        {
          return true;
        }
      }
    }

    switch_srv.request.start_controllers.push_back(controller_name);
    switch_srv.request.strictness = switch_srv.request.BEST_EFFORT;

    if (!switch_controllers_client_.call(switch_srv))
    {
      ROS_ERROR("Error calling the switch controllers server!");
      return false;
    }

    if (!switch_srv.response.ok)
    {
      ROS_ERROR("Failed to switch to controller %s!", controller_name.c_str());
      return false;
    }
    return true;
  }

  /*
    Loads the given controller in the PR2
  */
  bool ExclusiveControllerRunner::loadController(std::string controller_name)
  {
    pr2_mechanism_msgs::LoadController load_srv;

    if (controllerIsLoaded(controller_name))
    {
      return true;
    }

    load_srv.request.name = controller_name;
    if(!load_controllers_client_.call(load_srv))
    {
      ROS_ERROR("Error calling the load controller server!");
      return false;
    }

    if(load_srv.response.ok)
    {
      return true;
    }
    else
    {
      ROS_ERROR("Failed in loading the controller %s!", controller_name.c_str());
      return false;
    }
  }
}

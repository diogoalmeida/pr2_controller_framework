#include <manipulation_controller/manipulation_controller.hpp>

namespace manipulation {
  ManipulationController::ManipulationController()
  {
    nh_ = ros::NodeHandle("~");

    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    action_server_ = new actionlib::SimpleActionServer<manipulation_controller::ManipulationControllerAction>(nh_, action_name_, false);

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&ManipulationController::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&ManipulationController::preemptCB, this));

    action_server_->start();
  }

  /*
    Receive a new actiongoal: update controller input parameters.
  */
  void ManipulationController::goalCB()
  {

  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ManipulationController::publishFeedback()
  {

  }

  /*
    Search for controller relevant parameters in the parameter server
  */
  bool ManipulationController::loadParams()
  {
    if (!nh_.getParam("/manipulation_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/manipulation_controller/action_server_name)");
      return false;
    }
  }

  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;

    // Magic happens

    return control_output;
  }
}

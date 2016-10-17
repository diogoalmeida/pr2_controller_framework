#include <pr2_controller/joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {


/// Controller initialization in non-realtime
bool JointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("Joint controller could not find joint named '%s'", joint_name.c_str());
    return false;
  }

  // copy robot pointer so we can access time
  robot_ = robot;

  // construct pid controller
  if (!pid_controller_.init(ros::NodeHandle(n, "pid_parameters"))){
    ROS_ERROR("Joint controller could not construct PID controller for joint '%s'",
              joint_name.c_str());
    return false;
  }

  return true;
}

/// Controller startup in realtime
void JointController::starting()
{
  init_pos_ = joint_state_->position_;
  time_of_last_cycle_ = robot_->getTime();
  pid_controller_.reset();
}

/// Controller update loop in realtime
void JointController::update()
{
  double desired_pos = init_pos_ + 15 * sin(ros::Time::now().toSec());
  double current_pos = joint_state_->position_;

  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();
  joint_state_->commanded_effort_ = pid_controller_.computeCommand(current_pos-desired_pos, dt);
}

/// Controller stopping in realtime
void JointController::stopping()
{}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(pr2_controller, JointController,
                         pr2_joint_controller::JointController,
                         pr2_controller_interface::Controller)

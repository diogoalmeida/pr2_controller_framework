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

  if (!n.getParam("actionlib_server_name", action_name_))
  {
    ROS_WARN("Action name not set. Using default 'PR2_pos_control'");
    action_name_ = "PR2_pos_control";
  }

  action_server_ = new actionlib::SimpleActionServer<pr2_controller::PR2JointCommandAction>(n, action_name_, false);
  action_server_->registerGoalCallback(boost::bind(&JointController::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&JointController::preemptCB, this));

  return true;
}

/// Gets a goal request from actionlib
void JointController::goalCB()
{
  int j = 0;
  pr2_mechanism_model::JointState *joint_state;
  goal_ = action_server_->acceptNewGoal();

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  control_references_.name.clear();
  control_references_.position.clear();
  control_references_.velocity.clear();
  control_references_.effort.clear();

  for(std::vector<std::string>::const_iterator i = goal_->commanded_joint_state.name.begin(); i != goal_->commanded_joint_state.name.end(); i++)
  {
    if (goal_->use_current_position)
    {
      joint_state = robot_->getJointState(*i);

      if(!joint_state)
      {
        ROS_ERROR("Could not find joint %s", (*i).c_str());
        action_server_->setAborted();
        return;
      }
      else
      {
        control_references_.position.push_back(joint_state->position_);
      }
    }
    else
    {
      control_references_.position.push_back(goal_->commanded_joint_state.position[j]);
    }

    control_references_.velocity.push_back(goal_->commanded_joint_state.velocity[j]);
    control_references_.effort.push_back(goal_->commanded_joint_state.effort[j]);
    control_references_.name.push_back(*i);
    j++;
  }
}

/// Handles preemption requests from the actionlib client
void JointController::preemptCB()
{
    ROS_WARN("PR2 Joint Controller preempted!");
    action_server_->setPreempted();
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

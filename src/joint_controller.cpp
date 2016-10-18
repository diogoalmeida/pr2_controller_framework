#include <pr2_controller/joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {


/*
  Controller initialization
*/
bool JointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  // copy robot pointer so we can access time
  robot_ = robot;
  action_server_ = 0;
  ROS_INFO("Initializing joint controller! Namespace: %s", n.getNamespace().c_str());

  if (!n.getParam("actionlib_server_name", action_name_))
  {
    ROS_WARN("Action name not set. Using default 'PR2_pos_control' (%s/actionlib_server_name)", n.getNamespace().c_str());
    action_name_ = "PR2_pos_control";
  }

  if (!n.getParam("actionlib_feedback_rate", feedback_hz_))
  {
    ROS_WARN("Action feedback rate not set. Using defaul 10 Hz (%s/actionlib_feedback_rate)", n.getNamespace().c_str());
    feedback_hz_ = 10.0;
  }

  if (!n.getParam("actuated_joint_names", joint_names_))
  {
    ROS_ERROR("Joint controller requires a set of joint names (%s/actuated_joint_names)", n.getNamespace().c_str());
    return false;
  }

  if (joint_names_.size() == 0)
  {
    ROS_ERROR("Joint controller initialized with no joint names");
    return false;
  }

  ROS_INFO("Loaded joint names:");
  for(int i = 0; i < joint_names_.size(); i++) // initialize the joint controllers. Expecting one set of PID gains per actuated joint
  {
    ROS_INFO("%s", joint_names_[i].c_str());
    if(!n.hasParam("controller_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects controller gains for joint %s (%s/controller_gains/%s)", joint_names_[i].c_str(), n.getNamespace().c_str(), joint_names_[i].c_str());
      return false;
    }

    // create a controller instance and give it a unique namespace for setting the controller gains
    joint_controllers_.push_back(new control_toolbox::Pid());
    time_of_last_cycle_.push_back(robot_->getTime());
    joint_controllers_[i]->init(ros::NodeHandle(n, "controller_gains/" + joint_names_[i]));
  }

  action_server_ = new actionlib::SimpleActionServer<pr2_controller::PR2JointCommandAction>(n, action_name_, false);
  action_server_->registerGoalCallback(boost::bind(&JointController::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&JointController::preemptCB, this));
  action_server_->start();

  // launch feedback thread. Allows publishing feedback outside of the realtime loop
  boost::thread(boost::bind(&JointController::publishFeedback, this));

  return true;
}

/*
  Goal callback. It is expected that a goal will include a reference for each
  joint given in the parameter "actuated_joint_names", otherwise the request will
  be ignored and the server will enter an aborted state.
*/
void JointController::goalCB()
{
  int number_of_matching_joints = 0;
  pr2_mechanism_model::JointState *joint_state;
  goal_ = action_server_->acceptNewGoal();

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  if (goal_->commanded_joint_state.name.size() != goal_->commanded_joint_state.position.size()
      || goal_->commanded_joint_state.name.size() != goal_->commanded_joint_state.velocity.size()
      || goal_->commanded_joint_state.name.size() != goal_->commanded_joint_state.effort.size())
  {
    ROS_ERROR("Joint controller goal request does not have consistent size for name, position, velocity and effort");
    action_server_->setAborted();
    return;
  }

  control_references_.name.clear();
  control_references_.position.clear();
  control_references_.velocity.clear();
  control_references_.effort.clear();

  for (int i = 0; i < goal_->commanded_joint_state.name.size(); i++)
  {
    if (i >= joint_names_.size())
    {
      ROS_ERROR("Joint controller goal request does not set references for all the commanded joints");
      action_server_->setAborted();
      return;
    }

    if (isActuatedJoint(goal_->commanded_joint_state.name[i]))
    {
      number_of_matching_joints++;
      if (goal_->use_current_position)
      {
        joint_state = robot_->getJointState(goal_->commanded_joint_state.name[i]);

        if(!joint_state)
        {
          ROS_ERROR("Could not find joint %s", goal_->commanded_joint_state.name[i].c_str());
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
        control_references_.position.push_back(goal_->commanded_joint_state.position[i]);
      }

      control_references_.velocity.push_back(goal_->commanded_joint_state.velocity[i]);
      control_references_.effort.push_back(goal_->commanded_joint_state.effort[i]);
      control_references_.name.push_back(goal_->commanded_joint_state.name[i]);

      if (number_of_matching_joints == joint_names_.size())
      {
        ROS_INFO("Joint controller received a valid request");
        break;
      }
    }
  }

  if (number_of_matching_joints != joint_names_.size())
  {
    ROS_ERROR("Goal request does not set references to all the commanded joints. Got %d matching joints for %d actuated joints", number_of_matching_joints, (int)joint_names_.size());
    action_server_->setAborted();
    return;
  }
}

/// Handles preemption requests from the actionlib client
void JointController::preemptCB()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  control_references_.name.clear();
  control_references_.position.clear();
  control_references_.velocity.clear();
  control_references_.effort.clear();

  ROS_WARN("PR2 Joint Controller preempted!");
  action_server_->setPreempted();
}

/// Controller startup in realtime
void JointController::starting()
{
  for(int i = 0; i < joint_controllers_.size(); i++)
  {
    joint_controllers_[i]->reset();
    time_of_last_cycle_[i] = robot_->getTime();
  }
}

/// Controller update loop in realtime
void JointController::update()
{
  ros::Duration dt;
  pr2_mechanism_model::JointState *joint_state;
  double current_position = 0.0;
  double desired_position = 0.0;

  boost::mutex::scoped_lock lock(reference_mutex_, boost::try_to_lock); // If no lock is obtained, this means the controller is updating references and we will set the control to the current position
  if (lock && action_server_->isActive())
  {
    for (int i = 0; i < joint_controllers_.size(); i++)
    {
      joint_state = robot_->getJointState(joint_names_[i]);
      if(!joint_state)
      {
        ROS_ERROR("Failed to get joint state in joint controller update loop!! This should NEVER happen");
        break;
      }

      current_position = joint_state->position_;
      desired_position = getReferencePosition(joint_names_[i]);
      dt = robot_->getTime() - time_of_last_cycle_[i];
      joint_state->commanded_effort_ = joint_controllers_[i]->computeCommand(current_position - desired_position, dt);
    }
  }
  else
  {
    if(action_server_->isActive())
    {
      ROS_WARN("Joint controller lock fail (possibly updating references)");
    }

    for (int i = 0; i < joint_controllers_.size(); i++)
    {
      joint_state = robot_->getJointState(joint_names_[i]);
      if(!joint_state)
      {
        ROS_ERROR("Failed to get joint state in joint controller update loop!! This should NEVER happen");
        break;
      }

      dt = robot_->getTime() - time_of_last_cycle_[i];
      joint_state->commanded_effort_ = joint_controllers_[i]->computeCommand(0, dt); // keep the current position
    }
  }
}

/*
  Get the position value in the control references for the given joint
*/
double JointController::getReferencePosition(std::string joint_name)
{
  for (int i = 0; i < control_references_.name.size(); i++)
  {
    if (control_references_.name[i] == joint_name)
    {
      return control_references_.position[i];
    }
  }

  ROS_ERROR("No reference available for joint name %s. This should NEVER happen.", joint_name.c_str());
  return 0.0;
}

/*
  Check if the given joint name is actuated by the controller
*/
bool JointController::isActuatedJoint(std::string joint_name)
{
  return std::find(joint_names_.begin(), joint_names_.end(), joint_name) != joint_names_.end();
}

/*
  Publish feedback at a (non-realtime) rate
*/
void JointController::publishFeedback()
{
  ros::Rate feedback_rate(feedback_hz_);

  while(ros::ok())
  {
    feedback_.commanded_effort.clear();
    if (action_server_ != 0)
    {
      if (action_server_->isActive())
      {
        for (int i = 0; i < joint_controllers_.size(); i++)
        {
          feedback_.commanded_effort.push_back(joint_controllers_[i]->getCurrentCmd());
        }

        action_server_->publishFeedback(feedback_);
      }
    }

    feedback_rate.sleep();
  }
}

/*
  Controller stopping in realtime
*/
void JointController::stopping()
{
  for (int i = 0; i < joint_controllers_.size(); i++)
  {
    delete joint_controllers_[i];
  }

  delete action_server_;
  action_server_ = 0;
}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(pr2_controller, JointController,
                         pr2_joint_controller::JointController,
                         pr2_controller_interface::Controller)

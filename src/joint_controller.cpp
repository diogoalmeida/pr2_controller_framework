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
  reference_active_ = false;
  pr2_mechanism_model::JointState *joint;
  ROS_INFO("Initializing joint controller! Namespace: %s", n.getNamespace().c_str());

  if (!n.getParam("feedback_rate", feedback_hz_))
  {
    ROS_WARN("Joint controller feedback rate not set. Using defaul 10 Hz (%s/feedback_rate)", n.getNamespace().c_str());
    feedback_hz_ = 10.0;
  }

  if (!n.getParam("receive_position_reference", receive_pos_reference_))
  {
    ROS_WARN("Missing position reference setting. Defaulting to using a position reference (%s/receive_position_reference)", n.getNamespace().c_str());
    receive_pos_reference_ = true;
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
  for(int i = 0; i < joint_names_.size(); i++) // initialize the position joint controllers. Expecting one set of PID gains per actuated joint
  {
    ROS_INFO("%s", joint_names_[i].c_str());
    if(!n.hasParam("position_loop_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects velocity loop gains for joint %s (%s/position_loop_gains/%s)", joint_names_[i].c_str(), n.getNamespace().c_str(), joint_names_[i].c_str());
      return false;
    }

    // create a controller instance and give it a unique namespace for setting the controller gains
    position_joint_controllers_.push_back(new control_toolbox::Pid());
    modified_velocity_references_.push_back(0);
    time_of_last_cycle_.push_back(robot_->getTime());
    joint = robot_->getJointState(joint_names_[i]);

    if (!joint)
    {
      ROS_ERROR("Joint %s does not exist in the robot mechanism model!", joint_names_[i].c_str());
      return false;
    }

    last_active_joint_position_.push_back(joint->position_);
    position_joint_controllers_[i]->init(ros::NodeHandle(n, "position_loop_gains/" + joint_names_[i]));
  }

  for(int i = 0; i < joint_names_.size(); i++) // initialize the velocity joint controllers. Expecting one set of PID gains per actuated joint
  {
    if(!n.hasParam("velocity_loop_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects velocity loop gains for joint %s (%s/velocity_loop_gains/%s)", joint_names_[i].c_str(), n.getNamespace().c_str(), joint_names_[i].c_str());
      return false;
    }

    // create a controller instance and give it a unique namespace for setting the controller gains
    velocity_joint_controllers_.push_back(new control_toolbox::Pid());
    time_of_last_cycle_.push_back(robot_->getTime());
    velocity_joint_controllers_[i]->init(ros::NodeHandle(n, "velocity_loop_gains/" + joint_names_[i]));
  }

  feedback_pub_ = n.advertise<pr2_controller::PR2JointControllerFeedback>(n.getNamespace() + "/control_feedback", 1);

  // launch feedback thread. Allows publishing feedback outside of the realtime loop
  boost::thread(boost::bind(&JointController::publishFeedback, this));

  // subscribe to the reference topic
  reference_subscriber_ = n.subscribe(n.getNamespace() + "/control_reference", 1, &JointController::referenceCallback, this);

  double timeout = 0.0;
  if (!n.getParam("control_timeout", timeout))
  {
    ROS_WARN("No control timeout set, will default to 0.01 sec (%s/control_timeout)", n.getNamespace().c_str());
    timeout = 0.01;
  }

  control_timeout_ = ros::Duration(timeout);
  time_of_last_reference_update_ = robot_->getTime();

  return true;
}

/*
  Update the current controller reference. If no update is sent for a predefined
  ammount of time, abort.
*/
void JointController::referenceCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  int number_of_matching_joints = 0;
  pr2_mechanism_model::JointState *joint_state;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  if (msg->name.size() != msg->position.size()
      || msg->name.size() != msg->velocity.size()
      || msg->name.size() != msg->effort.size())
  {
    ROS_ERROR("Joint controller reference does not have consistent size for name, position, velocity and effort");
    reference_active_ = false;
    return;
  }

  control_references_.name.clear();
  control_references_.position.clear();
  control_references_.velocity.clear();
  control_references_.effort.clear();

  for (int i = 0; i < msg->name.size(); i++)
  {
    if (i >= joint_names_.size())
    {
      ROS_ERROR("Joint controller reference does not set references for all the commanded joints");
      reference_active_ = false;
      return;
    }

    if (isActuatedJoint(msg->name[i]))
    {
      number_of_matching_joints++;
      if (!receive_pos_reference_)
      {
        joint_state = robot_->getJointState(msg->name[i]); // joint_state name was verified in init()
        control_references_.position.push_back(joint_state->position_);
      }
      else
      {
        control_references_.position.push_back(msg->position[i]);
      }

      control_references_.velocity.push_back(msg->velocity[i]);
      control_references_.effort.push_back(msg->effort[i]);
      control_references_.name.push_back(msg->name[i]);

      if (number_of_matching_joints == joint_names_.size())
      {
        ROS_DEBUG("Joint controller received a valid request");
        break;
      }
    }
  }

  if (number_of_matching_joints != joint_names_.size())
  {
    ROS_ERROR("Reference does not set references to all the commanded joints. Got %d matching joints for %d actuated joints", number_of_matching_joints, (int)joint_names_.size());
    reference_active_ = false;
    return;
  }

  reference_active_ = true;
  time_of_last_reference_update_ = robot_->getTime();
}

/// Controller startup in realtime
void JointController::starting()
{
  for(int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    position_joint_controllers_[i]->reset();
    velocity_joint_controllers_[i]->reset();
    time_of_last_cycle_[i] = robot_->getTime();
  }
}

/// Controller update loop in realtime
void JointController::update()
{
  ros::Duration dt;
  pr2_mechanism_model::JointState *joint_state;

  if (robot_->getTime() - time_of_last_reference_update_ > control_timeout_)
  {
    if(reference_active_)
    {
      ROS_ERROR("Joint controller timed out! (no reference received)");
      reference_active_ = false;
    }
  }

  boost::mutex::scoped_lock lock(reference_mutex_, boost::try_to_lock); // If no lock is obtained, this means the controller is updating references and we will set the control to the current position
  if (lock && reference_active_)
  {
    for (int i = 0; i < velocity_joint_controllers_.size(); i++)
    {
      joint_state = robot_->getJointState(joint_names_[i]); // sanity of joint_names_ has been verified in init()
      dt = robot_->getTime() - time_of_last_cycle_[i];
      last_active_joint_position_[i] = joint_state->position_;
      joint_state->commanded_effort_ = applyControlLoop(joint_state, getReferencePosition(joint_names_[i]), getReferenceVelocity(joint_names_[i]), i, dt);
      joint_state->enforceLimits();
      time_of_last_cycle_[i] = robot_->getTime();
    }
  }
  else
  { // If the controller is not active, or the references are being updated, keep the last recorded position reference
    if(reference_active_)
    {
      ROS_WARN("Joint controller lock fail (possibly updating references)");
    }
    else
    {
      for (int i = 0; i < velocity_joint_controllers_.size(); i++)
      {
        joint_state = robot_->getJointState(joint_names_[i]); // sanity of joint_names_ has been verified in init()
        dt = robot_->getTime() - time_of_last_cycle_[i];
        joint_state->commanded_effort_ = applyControlLoop(joint_state, last_active_joint_position_[i], 0, i, dt);
        joint_state->enforceLimits();
        time_of_last_cycle_[i] = robot_->getTime();
      }
    }
  }
}

/*
  Apply position feedback, add it to the velocity reference (which acts as a
  feedforward term) and then apply the velocity feedback.
*/
double JointController::applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, int controller_num, ros::Duration dt)
{
  double current_position, position_error, position_feedback;
  double current_velocity, velocity_error;

  current_position = joint_state->position_;
  position_error = desired_position - current_position;

  current_velocity = joint_state->velocity_;
  position_feedback = position_joint_controllers_[controller_num]->computeCommand(position_error, dt);
  velocity_error = desired_velocity + position_feedback - current_velocity;
  modified_velocity_references_[controller_num] = desired_velocity + position_feedback;

  return velocity_joint_controllers_[controller_num]->computeCommand(velocity_error, dt);
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
  Get the velocity value in the control references for the given joint
*/
double JointController::getReferenceVelocity(std::string joint_name)
{
  for (int i = 0; i < control_references_.name.size(); i++)
  {
    if (control_references_.name[i] == joint_name)
    {
      return control_references_.velocity[i];
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
  pr2_mechanism_model::JointState *joint_state;

  while(ros::ok())
  {
    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      feedback_.commanded_effort.clear();
      feedback_.position_error.clear();
      feedback_.velocity_error.clear();
      for (int i = 0; i < control_references_.name.size(); i++)
      {
        joint_state = robot_->getJointState(control_references_.name[i]);
        feedback_.commanded_effort.push_back(joint_state->commanded_effort_);
        feedback_.position_error.push_back(joint_state->position_ - control_references_.position[i]);
        feedback_.velocity_error.push_back(joint_state->velocity_ - control_references_.velocity[i]);

        feedback_.velocity_error_norm = std::abs(joint_state->velocity_ - modified_velocity_references_[i]); // for now just keeping one value
        feedback_.position_error_norm = std::abs(joint_state->position_ - control_references_.position[i]);
        feedback_.effort_single =joint_state->commanded_effort_;
        feedback_.position_feedback_norm = std::abs(modified_velocity_references_[i] - control_references_.velocity[i]);
      }
    }

    if (reference_active_)
    {
      feedback_.active = true;
    }
    else
    {
      feedback_.active = false;
    }

    feedback_pub_.publish(feedback_);
    feedback_rate.sleep();
  }
}

/*
  Controller stopping in realtime
*/
void JointController::stopping()
{
  for (int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    delete velocity_joint_controllers_[i];
  }
}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(pr2_controller, JointController,
                         pr2_joint_controller::JointController,
                         pr2_controller_interface::Controller)

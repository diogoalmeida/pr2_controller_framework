#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

/*
  Controller initialization
*/
bool TemplateJointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  // copy robot pointer so we can access time
  robot_ = robot;
  reference_active_ = false;
  controller_is_loaded_ = false;
  pr2_mechanism_model::JointState *joint;
  ROS_INFO("Initializing joint controller! Namespace: %s", n.getNamespace().c_str());

  cartesian_controller_ = initializeController();

  if (!n.getParam("feedback_rate", feedback_hz_))
  {
    ROS_WARN("Joint controller feedback rate not set. Using defaul 10 Hz (%s/feedback_rate)", n.getNamespace().c_str());
    feedback_hz_ = 10.0;
  }

  if (!n.getParam("/common/actuated_joint_names", joint_names_))
  {
    ROS_ERROR("Joint controller requires a set of joint names (/common/actuated_joint_names)");
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
    if(!n.hasParam("/common/position_loop_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects velocity loop gains for joint %s (/common/position_loop_gains/%s)", joint_names_[i].c_str(), joint_names_[i].c_str());
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
    position_joint_controllers_[i]->init(ros::NodeHandle(n, "/common/position_loop_gains/" + joint_names_[i]));
  }

  for(int i = 0; i < joint_names_.size(); i++) // initialize the velocity joint controllers. Expecting one set of PID gains per actuated joint
  {
    if(!n.hasParam("/common/velocity_loop_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects velocity loop gains for joint %s (/common/velocity_loop_gains/%s)", joint_names_[i].c_str(), joint_names_[i].c_str());
      return false;
    }

    // create a controller instance and give it a unique namespace for setting the controller gains
    velocity_joint_controllers_.push_back(new control_toolbox::Pid());
    time_of_last_cycle_.push_back(robot_->getTime());
    velocity_joint_controllers_[i]->init(ros::NodeHandle(n, "/common/velocity_loop_gains/" + joint_names_[i]));
  }

  feedback_pub_ = n.advertise<pr2_joint_position_controllers::PR2JointControllerFeedback>(n.getNamespace() + "/control_feedback", 1);

  time_of_last_reference_update_ = robot_->getTime();

  ROS_INFO("%s has loaded successfully!", n.getNamespace().c_str());

  return true;
}

/// Controller startup in realtime
void TemplateJointController::starting()
{
  controller_is_loaded_ = true;
  for(int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    position_joint_controllers_[i]->reset();
    velocity_joint_controllers_[i]->reset();
    time_of_last_cycle_[i] = robot_->getTime();
  }
  time_of_last_manipulation_call_ = robot_->getTime();

  // launch feedback thread. Allows publishing feedback outside of the realtime loop
  feedback_thread_ = boost::thread(boost::bind(&TemplateJointController::publishFeedback, this));
  // feedback_thread_.detach();
}

void TemplateJointController::stopping()
{
  controller_is_loaded_ = false;
  ROS_INFO("Joint controller stopping!");
  feedback_thread_.join();
  for(int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    delete position_joint_controllers_[i];
    delete velocity_joint_controllers_[i];
  }

  delete cartesian_controller_;
  ROS_INFO("Joint controller stopped successfully!");
}

/// Controller update loop in realtime
void TemplateJointController::update()
{
  ros::Duration dt;
  pr2_mechanism_model::JointState *joint_state;
  sensor_msgs::JointState current_state;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  for (int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    joint_state = robot_->getJointState(joint_names_[i]);
    current_state.name.push_back(joint_names_[i]);
    current_state.position.push_back(joint_state->position_);
    current_state.velocity.push_back(joint_state->velocity_);
    current_state.effort.push_back(joint_state->measured_effort_);
  }

  current_state.header.stamp = robot_->getTime();
  dt = robot_->getTime() - time_of_last_manipulation_call_;
  control_references_ = cartesian_controller_->updateControl(current_state, dt);

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

/*
  Apply position feedback, add it to the velocity reference (which acts as a
  feedforward term) and then apply the velocity feedback.
*/
double TemplateJointController::applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, int controller_num, ros::Duration dt)
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
double TemplateJointController::getReferencePosition(std::string joint_name)
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
double TemplateJointController::getReferenceVelocity(std::string joint_name)
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
bool TemplateJointController::isActuatedJoint(std::string joint_name)
{
  return std::find(joint_names_.begin(), joint_names_.end(), joint_name) != joint_names_.end();
}

/*
  Publish feedback at a (non-realtime) rate
*/
void TemplateJointController::publishFeedback()
{
  pr2_mechanism_model::JointState *joint_state;

  ROS_INFO("FEEDBACK THREAD STARTED");

  while(controller_is_loaded_)
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
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
  }

  ROS_INFO("FEEDBACK THREAD DESTROYED");
}
} // namespace

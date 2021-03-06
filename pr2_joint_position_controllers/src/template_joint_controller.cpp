#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

bool TemplateJointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  try
  {
    // copy robot pointer so we can access time
    robot_ = robot;
    n_ = n;
    ROS_INFO("Initializing joint controller! Namespace: %s", n.getNamespace().c_str());

    if (!n.getParam("feedback_rate", feedback_hz_))
    {
      ROS_WARN("Joint controller feedback rate not set. Using default 10 Hz (%s/feedback_rate)", n.getNamespace().c_str());
      feedback_hz_ = 10.0;
    }

    if (allocateVariables())
    {
      return true;
    }

    ROS_ERROR("Something went wrong with variable allocation");
    return false;
  }
  catch (const std::exception &exc)
  {
    ROS_ERROR("Exception thrown: %s", exc.what());
    return false;
  }
  catch (...)
  {
    ROS_ERROR("Unknown exception!!");
    return false;
  }
}

void TemplateJointController::starting()
{
  cartesian_controller_ = initializeController();
  
  pr2_mechanism_model::JointState *joint_state;
  for(int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    joint_state = robot_->getJointState(joint_names_[i]);
    position_joint_controllers_[i]->reset();
    velocity_joint_controllers_[i]->reset();
    last_active_joint_position_[i] = joint_state->position_;
    time_of_last_cycle_[i] = robot_->getTime();
    modified_velocity_references_[i] = 0.0;
  }
  time_of_last_manipulation_call_ = robot_->getTime();

  // launch feedback thread. Allows publishing feedback outside of the realtime loop
  feedback_thread_ = boost::thread(boost::bind(&TemplateJointController::publishFeedback, this));
  ROS_INFO("Joint controller started!");
}

void TemplateJointController::stopping()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);

  if(feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }

  ROS_INFO("Joint controller stopped successfully!");
}

void TemplateJointController::update()
{
  ros::Duration dt;
  pr2_mechanism_model::JointState *joint_state;
  sensor_msgs::JointState current_state;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  for (int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    joint_state = robot_->getJointState(joint_names_[i]);
    if (!joint_state)
    {
      ROS_ERROR("Joint %s state in robot mechanism didn't return. This should NEVER happen", joint_names_[i].c_str());
      return; // skip a time step and hope for the best
    }
    current_state.name.push_back(joint_names_[i]);
    current_state.position.push_back(joint_state->position_);
    current_state.velocity.push_back(joint_state->velocity_);
    current_state.effort.push_back(joint_state->measured_effort_);
  }

  current_state.header.stamp = robot_->getTime();
  dt = robot_->getTime() - time_of_last_manipulation_call_;
  control_references_ = cartesian_controller_->updateControl(current_state, dt);
  time_of_last_manipulation_call_ = robot_->getTime();

  verifySanity(control_references_);

  for (int i = 0; i < velocity_joint_controllers_.size(); i++)
  {
    joint_state = robot_->getJointState(joint_names_[i]); // sanity of joint_names_ has been verified in init()
    if (!joint_state)
    {
      ROS_ERROR("Joint %s state in robot mechanism didn't return. This should NEVER happen", joint_names_[i].c_str());
      return; // skip a time step and hope for the best
    }
    dt = robot_->getTime() - time_of_last_cycle_[i];
    last_active_joint_position_[i] = joint_state->position_;
    joint_state->commanded_effort_ = applyControlLoop(joint_state, getReferencePosition(joint_names_[i]), getReferenceVelocity(joint_names_[i]), i, dt);
    joint_state->enforceLimits();
    time_of_last_cycle_[i] = robot_->getTime();
  }
}

bool TemplateJointController::verifySanity(sensor_msgs::JointState &state)
{
  pr2_mechanism_model::JointState *joint_state;
  boost::shared_ptr<const urdf::JointLimits> limits;
  bool hit_limit = false;

  for (int i = 0; i < state.name.size(); i++)
  {
    joint_state = robot_->getJointState(state.name[i]);

    if (!joint_state)
    {
      ROS_WARN("Tried to verify sanity to an inexistent joint: %s", state.name[i].c_str());
      continue;
    }

    limits = joint_state->joint_->limits;

    if (limits->upper != limits->lower) // it these limits are equal, joint can rotate freely
    {
      if (state.position[i] > limits->upper)
      {
        state.position[i] = limits->upper;
        hit_limit = true;
        ROS_WARN("Joint %s has a commanded position <%.2f> above the upper limit <%.2f>", state.name[i].c_str(), state.position[i], limits->upper);
      }

      if (state.position[i] < limits->lower)
      {
        state.position[i] = limits->lower;
        hit_limit = true;
        ROS_WARN("Joint %s has a commanded position <%.2f> bellow the lower limit <%.2f>", state.name[i].c_str(), state.position[i], limits->lower);
      }
    }
    
    if (std::abs(state.velocity[i]) > max_joint_velocity_)
    {
      if (state.velocity[i] < 0)
      {
        state.velocity[i] = -max_joint_velocity_;
      }
      else
      {
        state.velocity[i] = max_joint_velocity_;
      }
      
      ROS_WARN("Joint %s has a commanded velocity <%.2f> of higher magnitude than the manual velocity threshold <%.2f>", state.name[i].c_str(), state.velocity[i], max_joint_velocity_);
      hit_limit = true;
    }
    
    if (std::abs(state.velocity[i]) > limits->velocity)
    {
      if (state.velocity[i] < 0)
      {
        state.velocity[i] = -limits->velocity;
      }
      else
      {
        state.velocity[i] = limits->velocity;
      }

      hit_limit = true;
      ROS_WARN("Joint %s has a commanded velocity <%.2f> of higher magnitude than the limit <%.2f>", state.name[i].c_str(), state.velocity[i], limits->velocity);
    }
  }

  return !hit_limit;
}

void TemplateJointController::resetAllocableVariables()
{
  position_joint_controllers_.clear();
  velocity_joint_controllers_.clear();
  joint_names_.clear();
  time_of_last_cycle_.clear();
  ff_joint_controllers_.clear();
  last_active_joint_position_.clear();
  modified_velocity_references_.clear();
}

bool TemplateJointController::allocateVariables()
{
  pr2_mechanism_model::JointState *joint;
  
  if (!cartesian_controller_)
  {
    cartesian_controller_ = initializeController();
  }

  if (!n_.getParam("/common/actuated_joint_names", joint_names_))
  {
    ROS_ERROR("Joint controller requires a set of joint names (/common/actuated_joint_names)");
    return false;
  }
  
  if (!n_.getParam("/common/max_joint_velocity", max_joint_velocity_))
  {
    ROS_ERROR("Joint controller requires a joint velocity saturation value (/common/max_joint_velocity)");
    return false;
  }
  
  if (joint_names_.size() == 0)
  {
    ROS_ERROR("Joint controller initialized with no joint names");
    return false;
  }
  
  position_joint_controllers_.resize(joint_names_.size());
  velocity_joint_controllers_.resize(joint_names_.size());
  time_of_last_cycle_.resize(joint_names_.size());
  ff_joint_controllers_.resize(joint_names_.size());
  last_active_joint_position_.resize(joint_names_.size());
  modified_velocity_references_.resize(joint_names_.size());

  for(int i = 0; i < joint_names_.size(); i++) // initialize the position joint controllers. Expecting one set of PID gains per actuated joint
  {
    if(!n_.hasParam("/common/position_loop_gains/" + joint_names_[i]))
    {
      ROS_ERROR("Joint controller expects position loop gains for joint %s (/common/position_loop_gains/%s)", joint_names_[i].c_str(), joint_names_[i].c_str());
      return false;
    }

    ROS_DEBUG("Pushing a controller for joint %s.", joint_names_[i].c_str());

    // create a controller instance and give it a unique namespace for setting the controller gains
    position_joint_controllers_[i] = boost::shared_ptr<control_toolbox::Pid>(new control_toolbox::Pid());
    modified_velocity_references_[i] = 0.0;
    joint = robot_->getJointState(joint_names_[i]);

    if (!joint)
    {
      ROS_ERROR("Joint %s does not exist in the robot mechanism model!", joint_names_[i].c_str());
      return false;
    }
    else
    {
      ROS_DEBUG("Allocating a controller for joint %s.", joint_names_[i].c_str());
    }

    last_active_joint_position_[i] = joint->position_;
    position_joint_controllers_[i]->init(ros::NodeHandle(n_, "/common/position_loop_gains/" + joint_names_[i]));
  }

  double ff_gain;
  for(int i = 0; i < joint_names_.size(); i++) // initialize the velocity joint controllers. Expecting one set of PID gains per actuated joint
  {
    if(!n_.hasParam("/common/velocity_loop_gains/" + joint_names_[i])) // I'm trusting the user to actually set the gains on this namespace... otherwise, it will use the dynamic reconfig defaults
    {
      ROS_ERROR("Joint controller expects velocity loop gains for joint %s (/common/velocity_loop_gains/%s)", joint_names_[i].c_str(), joint_names_[i].c_str());
      return false;
    }

    if(!n_.getParam("/common/velocity_loop_gains/" + joint_names_[i] + "/feedforward_gain", ff_gain))
    {
      ROS_ERROR("Joint controller expects a velocity feedforward gain for joint %s (/common/velocity_loop_gains/%s/feedforward_gain)", joint_names_[i].c_str(), joint_names_[i].c_str());
      return false;
    }

    // create a controller instance and give it a unique namespace for setting the controller gains
    velocity_joint_controllers_[i]= boost::shared_ptr<control_toolbox::Pid>(new control_toolbox::Pid());
    time_of_last_cycle_[i]= robot_->getTime();
    ff_joint_controllers_[i]= ff_gain;
    velocity_joint_controllers_[i]->init(ros::NodeHandle(n_, "/common/velocity_loop_gains/" + joint_names_[i]));
  }

  feedback_pub_ = n_.advertise<pr2_joint_position_controllers::PR2JointControllerFeedback>(n_.getNamespace() + "/control_feedback", 1);
  time_of_last_reference_update_ = robot_->getTime();
  ROS_INFO("%s has loaded successfully!", n_.getNamespace().c_str());

  return true;
}

double TemplateJointController::applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, int controller_num, ros::Duration dt)
{
  double current_position, position_error, position_feedback;
  double current_velocity, velocity_error;
  
  //TODO: Make proper
  bool is_old = false;
  
  if (is_old)
  {
    current_position = joint_state->position_;
    position_error = desired_position - current_position;
    current_velocity = joint_state->velocity_;
    position_feedback = position_joint_controllers_[controller_num]->computeCommand(position_error, dt);
    velocity_error = desired_velocity + position_feedback - current_velocity;
    modified_velocity_references_[controller_num] = desired_velocity + position_feedback;
    
    return velocity_joint_controllers_[controller_num]->computeCommand(velocity_error, dt) + ff_joint_controllers_[controller_num]*desired_velocity;
  }
  
  current_position = joint_state->position_;
  current_velocity = joint_state->velocity_;
  position_error = desired_position - current_position;
  velocity_error = desired_velocity - current_velocity;
  
  return position_joint_controllers_[controller_num]->computeCommand(position_error, velocity_error, dt);
}

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

bool TemplateJointController::isActuatedJoint(std::string joint_name)
{
  return std::find(joint_names_.begin(), joint_names_.end(), joint_name) != joint_names_.end();
}

void TemplateJointController::publishFeedback()
{
  pr2_mechanism_model::JointState *joint_state;

  try
  {
    while(ros::ok())
    {
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        if (verifySanity(control_references_))
        {
          feedback_.joint_name.clear();
          feedback_.commanded_effort.clear();
          feedback_.position_error.clear();
          feedback_.velocity_error.clear();
          feedback_.position.clear();
          feedback_.velocity.clear();
          feedback_.velocity_nominal_error.clear();

          for (int i = 0; i < control_references_.name.size(); i++)
          {
            joint_state = robot_->getJointState(control_references_.name[i]);
            if (!joint_state)
            {
              ROS_ERROR("Joint %s state in robot mechanism didn't return. This should NEVER happen", control_references_.name[i].c_str());
              continue; // skip a time step and hope for the best
            }
            feedback_.joint_name.push_back(control_references_.name[i]);
            feedback_.commanded_effort.push_back(joint_state->commanded_effort_);
            feedback_.position_error.push_back(joint_state->position_ - control_references_.position[i]);
            feedback_.velocity_error.push_back(joint_state->velocity_ - control_references_.velocity[i]);
            feedback_.velocity_nominal_error.push_back(joint_state->velocity_ - modified_velocity_references_[i]);
            feedback_.position.push_back(joint_state->position_);
            feedback_.velocity.push_back(joint_state->velocity_);

            feedback_.velocity_error_norm = std::abs(joint_state->velocity_ - modified_velocity_references_[i]); // for now just keeping one value
            feedback_.position_error_norm = std::abs(joint_state->position_ - control_references_.position[i]);
            feedback_.effort_single = joint_state->commanded_effort_;
            feedback_.position_feedback_norm = std::abs(modified_velocity_references_[i] - control_references_.velocity[i]);
          }
        feedback_pub_.publish(feedback_);
        }
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
    }
  }
  catch(const boost::thread_interrupted &)
  {
    ROS_DEBUG("Joint controller thread interrupted :)");
    return;
  }
}
} // namespace

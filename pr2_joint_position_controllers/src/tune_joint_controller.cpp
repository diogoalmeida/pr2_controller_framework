#include <pr2_joint_position_controllers/tune_joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

/*
  Controller initialization
*/
bool TuneJointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  // copy robot pointer so we can access time
  robot_ = robot;
  ROS_INFO("Initializing joint controller! Namespace: %s", n.getNamespace().c_str());

  if (!n.getParam("feedback_rate", feedback_hz_))
  {
    ROS_WARN("Joint controller feedback rate not set. Using defaul 10 Hz (%s/feedback_rate)", n.getNamespace().c_str());
    feedback_hz_ = 10.0;
  }

  if (!n.getParam("action_name", action_name_))
  {
    ROS_WARN("Joint controller action_name not set. Aborting (%s/action_name)", n.getNamespace().c_str());
    return false;
  }

  position_joint_controller_ = 0;
  velocity_joint_controller_ = 0;

  action_server_ = new actionlib::SimpleActionServer<pr2_joint_position_controllers::PR2TuneJointAction>(n, action_name_, false);

  action_server_->registerGoalCallback(boost::bind(&TuneJointController::goalCallback, this));
  action_server_->registerPreemptCallback(boost::bind(&TuneJointController::preemptCallback, this));
  n_ = n;

  ROS_INFO("%s has loaded successfully!", n.getNamespace().c_str());

  return true;
}

/*
  Accept a new goal consisting of the joint we want to tune and
  desired PID parameters
*/
void TuneJointController::goalCallback()
{
  ROS_INFO("%s action server received a new goal!", action_name_.c_str());

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  boost::shared_ptr<const pr2_joint_position_controllers::PR2TuneJointGoal> goal = action_server_->acceptNewGoal();

  joint_name_ = goal->joint;
  pr2_mechanism_model::JointState *joint_state = robot_->getJointState(joint_name_);

  if (!joint_state)
  {
    ROS_ERROR("Joint %s does not exist in the robot mechanism model!", joint_name_.c_str());
    action_server_->setAborted();
  }
  else
  {
    control_reference_.name.clear();
    control_reference_.position.clear();
    control_reference_.velocity.clear();
    control_reference_.effort.clear();

    control_reference_.name.push_back(goal->joint);
    control_reference_.position.push_back(goal->target_position);
    control_reference_.velocity.push_back(goal->target_velocity);
    control_reference_.effort.push_back(0.0);

    max_time_ = ros::Duration(goal->max_time);
    start_time_ = robot_->getTime();
    position_joint_controller_ = new control_toolbox::Pid();
    position_joint_controller_->initPid(goal->gains_position.p, goal->gains_position.i, goal->gains_position.d, goal->gains_position.i_clamp, -goal->gains_position.i_clamp);
    velocity_joint_controller_ = new control_toolbox::Pid();
    velocity_joint_controller_->initPid(goal->gains_velocity.p, goal->gains_velocity.i, goal->gains_velocity.d, goal->gains_velocity.i_clamp, -goal->gains_velocity.i_clamp);
    ff_ = goal->velocity_ff;
    ROS_INFO("%s action server started!", action_name_.c_str());
  }
}

/*
  Preempt controller.
*/
void TuneJointController::preemptCallback()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  pr2_mechanism_model::JointState *joint_state = robot_->getJointState(joint_name_);
  result_.time_elapsed = (robot_->getTime() - start_time_).toSec();
  result_.final_error = control_reference_.position[0] - joint_state->position_;
  delete position_joint_controller_;
  delete velocity_joint_controller_;
  position_joint_controller_ = 0;
  velocity_joint_controller_ = 0;
  action_server_->setPreempted(result_);
  ROS_WARN("%s action server preempted!", action_name_.c_str());
}

/// Controller startup in realtime
void TuneJointController::starting()
{
  // launch feedback thread. Allows publishing feedback outside of the realtime loop
  action_server_->start();
  feedback_thread_ = boost::thread(boost::bind(&TuneJointController::publishFeedback, this));
  // feedback_thread_.detach();
}

void TuneJointController::stopping()
{
  ROS_INFO("Joint controller stopping!");
  if(feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }
  if (position_joint_controller_)
  {
    delete position_joint_controller_;
  }
  if (velocity_joint_controller_)
  {
    delete velocity_joint_controller_;
  }
  action_server_->shutdown();
  delete action_server_;
  ROS_INFO("Joint controller stopped successfully!");
}

/// Controller update loop in realtime
void TuneJointController::update()
{
  ros::Duration dt;
  pr2_mechanism_model::JointState *joint_state;
  sensor_msgs::JointState current_state;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  if (action_server_->isActive())
  {
    joint_state = robot_->getJointState(joint_name_);

    dt = robot_->getTime() - time_of_last_cycle_;
    joint_state->commanded_effort_ = applyControlLoop(joint_state, control_reference_.position[0], control_reference_.velocity[0], dt);
    joint_state->enforceLimits();
    time_of_last_cycle_ = robot_->getTime();

    if (robot_->getTime() - start_time_ > max_time_)
    {
      result_.time_elapsed = (robot_->getTime() - start_time_).toSec();
      result_.final_error = control_reference_.position[0] - joint_state->position_;
      action_server_->setSucceeded(result_);
      ROS_INFO("%s action server executed successfully!", action_name_.c_str());
    }
  }
}

/*
  Apply position feedback, add it to the velocity reference (which acts as a
  feedforward term) and then apply the velocity feedback.
*/
double TuneJointController::applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, ros::Duration dt)
{
  double current_position, position_error, position_feedback;
  double current_velocity, velocity_error;

  current_position = joint_state->position_;
  position_error = desired_position - current_position;

  current_velocity = joint_state->velocity_;
  position_feedback = position_joint_controller_->computeCommand(position_error, dt);
  velocity_error = desired_velocity + position_feedback - current_velocity;
  modified_velocity_reference_ = desired_velocity + position_feedback;

  return velocity_joint_controller_->computeCommand(velocity_error, dt) + ff_*desired_velocity;
}

/*
  Publish feedback at a (non-realtime) rate
*/
void TuneJointController::publishFeedback()
{
  pr2_mechanism_model::JointState *joint_state;

  ROS_INFO("FEEDBACK THREAD STARTED");

  try
  {
    while(ros::ok())
    {
      if (action_server_->isActive())
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        joint_state = robot_->getJointState(joint_name_);
        feedback_.joint_name = joint_name_;
        feedback_.commanded_effort = joint_state->commanded_effort_;
        feedback_.position_error = joint_state->position_ - control_reference_.position[0];
        feedback_.position_reference = control_reference_.position[0];
        feedback_.position = joint_state->position_;
        feedback_.velocity = joint_state->velocity_;
        feedback_.velocity_reference = modified_velocity_reference_;
        feedback_.velocity_error = modified_velocity_reference_ - joint_state->velocity_;
        action_server_->publishFeedback(feedback_);
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
    }
  }
  catch(const boost::thread_interrupted &)
  {
    ROS_INFO("FEEDBACK THREAD DESTROYED");
  }
}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::TuneJointController, pr2_controller_interface::Controller)

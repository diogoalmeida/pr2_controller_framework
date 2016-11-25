#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace cartesian_controllers {
  /*
    Receive a new actiongoal: update controller input parameters.
  */
  void ApproachController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::GuardedApproachGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::TwistStamped twist;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    twist = goal->approach_command;
    tf::twistMsgToKDL(twist.twist, velocity_reference_);

    force_threshold_ = goal->contact_force;
  }

  /*
    Preempt controller.
  */
  void ApproachController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ApproachController::publishFeedback()
  {
    feedback_.current_wrench.header.frame_id = base_link_;

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          feedback_.current_wrench.header.stamp = ros::Time::now();
          tf::wrenchEigenToMsg(measured_wrench_, feedback_.current_wrench.wrench);
          action_server_->publishFeedback(feedback_);
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
      }
    }
    catch(const boost::thread_interrupted &)
    {
      return;
    }
  }

  /*
    Search for controller relevant parameters in the parameter server
  */
  bool ApproachController::loadParams()
  {
    if (!nh_.getParam("/approach_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/approach_controller/action_server_name)");
      return false;
    }

    return true;
  }

  /*
    Implements the control strategy. This method is expected to call at a rate of approximately 1000 Hz. It should never
    take more than 1ms to execute.
  */
  sensor_msgs::JointState ApproachController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::JntArray commanded_joint_velocities;

    if (!action_server_->isActive())
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    if (measured_wrench_.block<3,1>(0,0).norm() > force_threshold_)
    {
      action_server_->setSucceeded(result_, "contact force achieved");
      return current_state;
    }

    for (int i = 0; i < 7; i++)
    {
      joint_positions_(i) = current_state.position[i];
    }

    ikvel_->CartToJnt(joint_positions_, velocity_reference_, commanded_joint_velocities);

    control_output = current_state;

    for (int i = 0; i < 7; i++)
    {
      control_output.velocity[i] = commanded_joint_velocities(i);
    }

    return control_output;
  }
}

#include <pr2_cartesian_controllers/move_controller.hpp>

namespace cartesian_controllers {
  /*
    Preempt controller.
  */
  void MoveController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
  }

  /*
    Receive a new actiongoal: update controller input parameters.
  */
  void MoveController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::MoveGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::PoseStamped pose;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    pose = goal->desired_pose;
    tf::poseMsgToKDL(pose.pose, pose_reference_);
  }

  /*
    Search for controller relevant parameters in the parameter server
  */
  bool MoveController::loadParams()
  {
    if (!nh_.getParam("/move_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/move_controller/action_server_name)");
      return false;
    }

    return true;
  }

  /*
    Implements the control strategy. This method is expected to call at a rate of approximately 1000 Hz. It should never
    take more than 1ms to execute.
  */
  sensor_msgs::JointState MoveController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::JntArray desired_joint_positions;

    if (!action_server_->isActive())
    {
      return current_state;
    }

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    for (int i = 0; i < 7; i++)
    {
      joint_positions_(i) = current_state.position[i];
    }

    ikpos_->CartToJnt(joint_positions_, pose_reference_, desired_joint_positions);

    control_output = current_state;

    for (int i = 0; i < 7; i++)
    {
      control_output.position[i] = desired_joint_positions(i);
    }

    return control_output;
  }
}

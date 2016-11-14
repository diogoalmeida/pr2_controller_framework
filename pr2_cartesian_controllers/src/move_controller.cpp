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
    Asynchronously publish a feedback message on the control status
  */
  void MoveController::publishFeedback()
  {
    ros::Rate feedback_rate(feedback_hz_);
    visualization_msgs::Marker reference_pose, current_pose;
    std_msgs::ColorRGBA object_color;
    Eigen::Vector3d r_1;
    KDL::Frame current_eef;

    object_color.r = 1;
    object_color.g = 0;
    object_color.b = 0;
    object_color.a = 1;

    reference_pose.header.frame_id = base_link_;
    reference_pose.header.stamp = ros::Time::now();
    reference_pose.ns = "move_controller";
    reference_pose.id = 1;
    reference_pose.type = reference_pose.SPHERE;
    reference_pose.action = reference_pose.ADD;
    reference_pose.color = object_color;
    reference_pose.lifetime = ros::Duration(0);
    reference_pose.frame_locked = false; // not sure about this
    reference_pose.scale.x = 0.1;
    reference_pose.scale.y = 0.1;
    reference_pose.scale.z = 0.1;

    current_pose = reference_pose;
    current_pose.id = 2;
    object_color.r = 0;
    object_color.g = 1;
    current_pose.color = object_color;


    while(ros::ok())
    {
      if (action_server_->isActive())
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        fkpos_->JntToCart(joint_positions_, current_eef);
        tf::poseKDLToMsg(pose_reference_, reference_pose.pose);
        tf::poseKDLToMsg(current_eef, current_pose.pose);

        target_pub_.publish(reference_pose);
        current_pub_.publish(current_pose);
      }
      feedback_rate.sleep();
    }
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
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

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

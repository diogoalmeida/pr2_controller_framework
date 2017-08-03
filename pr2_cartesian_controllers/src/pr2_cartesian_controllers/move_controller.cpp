#include <pr2_cartesian_controllers/move_controller.hpp>

namespace cartesian_controllers {
  MoveController::MoveController() : ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                        pr2_cartesian_controllers::MoveFeedback,
                                        pr2_cartesian_controllers::MoveResult>(),
    ik_service_name_(NUM_ARMS),
    ik_info_service_name_(NUM_ARMS),
    pose_reference_(NUM_ARMS),
    desired_joint_positions_(NUM_ARMS)
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    finished_acquiring_goal_ = false;
    is_single_arm_ = true;
    startActionlib();
    target_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_target", 1);
    current_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_current", 1);
    feedback_thread_ = boost::thread(boost::bind(&MoveController::publishFeedback, this));
  }

  MoveController::~MoveController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
    ROS_INFO("Shut down action server %s", action_name_.c_str());
  }

  void MoveController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    ROS_WARN("Move controller preempted!");
  }

  void MoveController::goalCB()
  {
    std::vector<geometry_msgs::PoseStamped> pose(NUM_ARMS);

    // It will take time to acquire the new joint positions, and I will have
    // accepted the goal already. This is a limitation of the goal callback
    // method. Since getting the desired joint positions require a service call,
    // it should be safer to not lock the reference mutex during the call, but
    // then I must make sure that the update control method doesn't use the
    // desired joint positions until they are set properly.
    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      finished_acquiring_goal_ = false;
    }

    boost::shared_ptr<const pr2_cartesian_controllers::MoveGoal> goal = action_server_->acceptNewGoal();

    if(goal->arm < 0 || goal->arm >= NUM_ARMS)
    {
      ROS_ERROR("Received a goal where the requested arm (%d) does not exist", goal->arm);
      action_server_->setAborted();
      return;
    }

    is_single_arm_ = goal->is_single_arm;

    if (is_single_arm_)
    {
      arm_index_ = goal->arm;
      for (int i = 0; i < NUM_ARMS; i++) // will be ignore later on; prevents ik failure
      {
        if (i != arm_index_)
        {
          pose[i] = pose[arm_index_];
        }
      }
    }

    ROS_DEBUG("Got arm %d", arm_index_);
    ROS_DEBUG("Available IK services:");
    for (int i = 0; i < NUM_ARMS; i++)
    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      pose[i] = goal->desired_pose[i];
      ROS_DEBUG("%s", ik_service_name_[i].c_str());
      ROS_DEBUG("%s", ik_info_service_name_[i].c_str());

      if(!getDesiredJointPositions(pose[i], desired_joint_positions_[i], actuated_joint_names_[i]))
      {
        action_server_->setAborted();
        {
          finished_acquiring_goal_ = true;
        }
        return;
      }

      tf::poseMsgToKDL(pose[i].pose, pose_reference_[i]);
      finished_acquiring_goal_ = true;
      loadParams();
      for (int j = 0; j < desired_joint_positions_[i].rows(); i++)
      {
        ROS_DEBUG("Position %d: %.2f", j, desired_joint_positions_[i](j));
      }
    }

    ROS_INFO("Move controller got a goal!");
  }

  bool MoveController::getDesiredJointPositions(const geometry_msgs::PoseStamped &pose, KDL::JntArray &joint_positions, std::vector<std::string> &joint_names)
  {
    moveit_msgs::GetPositionIK ik_srv;
    moveit_msgs::GetKinematicSolverInfo::Request info_request;
    moveit_msgs::GetKinematicSolverInfo::Response info_response;

    ROS_DEBUG("Waiting for %s", ik_service_name_[arm_index_].c_str());
    if (!ros::service::waitForService(ik_service_name_[arm_index_], ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to service %s", ik_service_name_[arm_index_].c_str());
      return false;
    }

    ROS_DEBUG("Waiting for %s", ik_info_service_name_[arm_index_].c_str());
    if (!ros::service::waitForService(ik_info_service_name_[arm_index_], ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to service %s", ik_info_service_name_[arm_index_].c_str());
      return false;
    }

    ROS_DEBUG("Creating service clients");
    ros::ServiceClient ik_client = nh_.serviceClient<moveit_msgs::GetPositionIK>(ik_service_name_[arm_index_]);
    ros::ServiceClient info_client = nh_.serviceClient<moveit_msgs::GetKinematicSolverInfo>(ik_info_service_name_[arm_index_]);

    ROS_DEBUG("Requesting info on the arm %d", arm_index_);

    if (!info_client.call(info_request, info_response))
    {
      ROS_ERROR("Error calling service %s", ik_info_service_name_[arm_index_].c_str());
      return false;
    }

    ik_srv.request.ik_request.ik_link_name = end_effector_link_[arm_index_];
    ik_srv.request.ik_request.pose_stamped = pose;
    ik_srv.request.ik_request.robot_state.joint_state.name = info_response.kinematic_solver_info.joint_names;
    ik_srv.request.ik_request.timeout = ros::Duration(5.0);

    for(unsigned int i = 0; i < info_response.kinematic_solver_info.joint_names.size(); i++)
    {
      ik_srv.request.ik_request.robot_state.joint_state.position.push_back((info_response.kinematic_solver_info.limits[i].min_position + info_response.kinematic_solver_info.limits[i].max_position)/2.0);
    }

    ROS_DEBUG("Requesting ik for the arm %d", arm_index_);

    if (!ik_client.call(ik_srv))
    {
      ROS_ERROR("Error calling service %s. Error code: %d", ik_service_name_[arm_index_].c_str(), ik_srv.response.error_code.val);
      return false;
    }

    if (ik_srv.response.error_code.val != ik_srv.response.error_code.SUCCESS)
    {
      ROS_ERROR("Error in service %s response. Code: %d", ik_service_name_[arm_index_].c_str(), ik_srv.response.error_code.val);
      return false;
    }

    ROS_DEBUG("Resizing joint positions");

    joint_names.clear();
    joint_positions.resize(ik_srv.response.solution.joint_state.position.size());

    for (int i = 0; i < ik_srv.response.solution.joint_state.position.size(); i++)
    {
      ROS_DEBUG("Assigning position %.2f", ik_srv.response.solution.joint_state.position[i]);
      joint_names.push_back(ik_srv.response.solution.joint_state.name[i]);
      joint_positions(i) = ik_srv.response.solution.joint_state.position[i];
    }

    return true;
  }

  bool MoveController::loadParams()
  {
    for (int i = 0; i < NUM_ARMS; i++)
    {
      if (!getParam("/move_controller/ik_service_name/arm_" + std::to_string(i + 1), ik_service_name_[i]))
      {
        return false;
      }

      if (!getParam("/move_controller/ik_info_service_name/arm_" + std::to_string(i + 1), ik_info_service_name_[i]))
      {
        return false;
      }
    }

    if (!getParam("/move_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/move_controller/max_allowed_error", max_allowed_error_))
    {
      return false;
    }

    if (!getParam("/move_controller/velocity_gain", velocity_gain_))
    {
      return false;
    }

    if (!getParam("/move_controller/error_threshold", error_threshold_))
    {
      return false;
    }

    return true;
  }

  void MoveController::publishFeedback()
  {
    if (!is_single_arm_) // NOT IMPLEMENTED
    {
      return;
    }

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
    reference_pose.type = reference_pose.ARROW;
    reference_pose.action = reference_pose.ADD;
    reference_pose.color = object_color;
    reference_pose.lifetime = ros::Duration(0);
    reference_pose.frame_locked = false; // not sure about this
    reference_pose.scale.x = 0.1;
    reference_pose.scale.y = 0.01;
    reference_pose.scale.z = 0.01;

    current_pose = reference_pose;
    current_pose.id = 2;
    object_color.r = 0;
    object_color.g = 1;
    current_pose.color = object_color;

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          if(finished_acquiring_goal_) // prevent accessing the joint positions_ vector when it can be uninitialized
          {
            // fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], current_eef);
            tf::poseKDLToMsg(pose_reference_[arm_index_], reference_pose.pose);
            tf::poseKDLToMsg(current_eef, current_pose.pose);

            target_pub_.publish(reference_pose);
            current_pub_.publish(current_pose);
            action_server_->publishFeedback(feedback_);
          }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
      }
    }
    catch(const boost::thread_interrupted &)
    {
      ROS_INFO("%s feedback thread interrupted", action_name_.c_str());
      return;
    }
  }

  double MoveController::getDesiredPosition(const std::string &joint_name)
  {

    if (is_single_arm_)
    {
      for (int i = 0; i < desired_joint_positions_[arm_index_].rows(); i++)
      {
        if (actuated_joint_names_[arm_index_][i] == joint_name)
        {
          return desired_joint_positions_[arm_index_](i);
        }
      }
    }
    else
    {
      for (int arm = 0; arm < NUM_ARMS; arm++)
      {
        for (int i = 0; i < desired_joint_positions_[arm].rows(); i++)
        {
          if (actuated_joint_names_[arm][i] == joint_name)
          {
            return desired_joint_positions_[arm](i);
          }
        }
      }
    }

    ROS_ERROR("Tried getting desired position for joint name %s, which should not be controlled.", joint_name.c_str());
    return 0.0;
  }

  sensor_msgs::JointState MoveController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output = current_state;

    if (!action_server_->isActive() || !finished_acquiring_goal_)
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;
    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    feedback_.joint_position_references.clear();
    feedback_.joint_position_errors.clear();
    feedback_.joint_velocity_references.clear();
    feedback_.joint_velocity_errors.clear();

    if (is_single_arm_)
    {
      if(!getChainJointState(current_state, chain_[arm_index_], joint_positions_[arm_index_], joint_velocities_[arm_index_]))
      {
        ROS_ERROR("Failed to get chain joint state");
        action_server_->setAborted();
        return lastState(current_state);
      }

      control_output = lastState(current_state, arm_index_);
    }
    else
    {
      for (int i = 0; i < NUM_ARMS; i++)
      {
        if(!getChainJointState(current_state, chain_[i], joint_positions_[i], joint_velocities_[i]))
        {
          ROS_ERROR("Failed to get chain joint state");
          action_server_->setAborted();
          return lastState(current_state);
        }
      }
      control_output = lastState(current_state);
    }

    // 0 - Check success
    int success = 0;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      if (!is_single_arm_ || hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        double val = getDesiredPosition(current_state.name[i]);
        if (std::abs(val - current_state.position[i]) > error_threshold_)
        {
          break;
        }
        else
        {
          success++;
        }
      }
    }

    int success_num;

    if (is_single_arm_)
    {
      success_num = chain_[arm_index_].getNrOfJoints();
    }
    else
    {
      success_num = NUM_ARMS*chain_[0].getNrOfJoints(); // Assumption: All arms have the same number of joints.
    }

    if (success == success_num)
    {
      action_server_->setSucceeded();
      return lastState(current_state);
    }


    // 1 - Compute position error
    for (int i = 0; i < current_state.name.size(); i++)
    {
      if (!is_single_arm_ || hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        double val = getDesiredPosition(current_state.name[i]);
        double e = 0;
        if (std::abs(val - current_state.position[i]) > max_allowed_error_)
        {
          if (val - current_state.position[i] > 0)
          {
            e = max_allowed_error_;
          }
          else
          {
            e = -max_allowed_error_;
          }
        }
        else
        {
          e = val - current_state.position[i];
        }

        control_output.velocity[i] = velocity_gain_ * e;
        control_output.position[i] = current_state.position[i] + velocity_gain_ * e * dt.toSec();

        feedback_.joint_position_references.push_back(val);
        feedback_.joint_position_errors.push_back(val - current_state.position[i]);
        feedback_.joint_velocity_references.push_back(velocity_gain_ * e);
        feedback_.joint_velocity_errors.push_back(velocity_gain_ * e - current_state.velocity[i]);
      }
      else
      {
        double val = control_output.position[i] - current_state.position[i];

        if (std::abs(val) > max_allowed_error_)
        {
          if (val > 0)
          {
            val = max_allowed_error_;
          }
          else
          {
            val = -max_allowed_error_;
          }
        }
        control_output.velocity[i] = velocity_gain_ * val;
      }
    }

    return control_output;
  }
}

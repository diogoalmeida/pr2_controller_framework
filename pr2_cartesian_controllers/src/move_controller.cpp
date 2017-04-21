#include <pr2_cartesian_controllers/move_controller.hpp>

namespace cartesian_controllers {
  MoveController::MoveController() : ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                        pr2_cartesian_controllers::MoveFeedback,
                                        pr2_cartesian_controllers::MoveResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    finished_acquiring_goal_ = false;
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
    delete action_server_;
  }

  void MoveController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    ROS_WARN("Move controller preempted!");
  }

  void MoveController::goalCB()
  {
    geometry_msgs::PoseStamped pose;
    KDL::JntArray temp_desired_joint_positions;

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

    arm_index_ = goal->arm;
    pose = goal->desired_pose;

    if(!getDesiredJointPositions(pose, temp_desired_joint_positions))
    {
      action_server_->setAborted();
      return;
    }

    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      tf::poseMsgToKDL(pose.pose, pose_reference_);
      desired_joint_positions_ = temp_desired_joint_positions;
      finished_acquiring_goal_ = true;
    }
    loadParams();
    ROS_INFO("Move controller got a goal!");
  }

  bool MoveController::getDesiredJointPositions(geometry_msgs::PoseStamped pose, KDL::JntArray &joint_positions)
  {
    moveit_msgs::GetPositionIK ik_srv;
    moveit_msgs::GetKinematicSolverInfo::Request info_request;
    moveit_msgs::GetKinematicSolverInfo::Response info_response;

    if (!ros::service::waitForService(ik_service_name_[arm_index_], ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to service %s", ik_service_name_[arm_index_].c_str());
      return false;
    }

    if (!ros::service::waitForService(ik_info_service_name_[arm_index_], ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to service %s", ik_info_service_name_[arm_index_].c_str());
      return false;
    }

    ros::ServiceClient ik_client = nh_.serviceClient<moveit_msgs::GetPositionIK>(ik_service_name_[arm_index_]);
    ros::ServiceClient info_client = nh_.serviceClient<moveit_msgs::GetKinematicSolverInfo>(ik_info_service_name_[arm_index_]);

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

    joint_positions.resize(ik_srv.response.solution.joint_state.position.size());

    for (int i = 0; i < ik_srv.response.solution.joint_state.position.size(); i++)
    {
      joint_positions(i) = ik_srv.response.solution.joint_state.position[i];
    }

    return true;
  }

  bool MoveController::loadParams()
  {
    for (int i = 0; i < NUM_ARMS; i++)
    {
      if (!getParam("/move_controller/ik_service_name/" + std::to_string(i + 1), ik_service_name_[i]))
      {
        return false;
      }

      if (!getParam("/move_controller/ik_info_service_name/" + std::to_string(i + 1), ik_info_service_name_[i]))
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
          fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], current_eef);
          tf::poseKDLToMsg(pose_reference_, reference_pose.pose);
          tf::poseKDLToMsg(current_eef, current_pose.pose);

          target_pub_.publish(reference_pose);
          current_pub_.publish(current_pose);
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

  sensor_msgs::JointState MoveController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;

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

    for (int i = 0; i < current_state.name.size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        joint_positions_[arm_index_](i) = current_state.position[i];
      }
    }

    control_output = current_state;

    // 0 - Check success
    int success = 0;
    for (int i = 0; current_state.name.size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        if (std::abs(desired_joint_positions_(i) - current_state.position[i]) > error_threshold_)
        {
          break;
        }
        else
        {
          success++;
        }
      }
    }

    if (success == chain_[arm_index_].getNrOfJoints())
    {
      ROS_INFO("Move joint controller executed successfully!");
      action_server_->setSucceeded();
      return lastState(current_state);
    }

    // 1 - Compute position error
    std::vector<double> error;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        if (std::abs(desired_joint_positions_(i) - current_state.position[i]) > max_allowed_error_)
        {
          if (desired_joint_positions_(i) - current_state.position[i] > 0)
          {
            error.push_back(max_allowed_error_);
          }
          else
          {
            error.push_back(-max_allowed_error_);
          }
        }
        else
        {
          error.push_back(desired_joint_positions_(i) - current_state.position[i]);
        }

        feedback_.joint_position_references.push_back(desired_joint_positions_(i));
        feedback_.joint_position_errors.push_back(desired_joint_positions_(i) - current_state.position[i]);
      }
    }

    // 2 - send commands
    for (int i = 0; i < current_state.name[i].size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        control_output.velocity[i] = velocity_gain_ * error[i];
        control_output.position[i] = current_state.position[i];
        feedback_.joint_velocity_references.push_back(velocity_gain_ * error[i]);
        feedback_.joint_velocity_errors.push_back(velocity_gain_ * error[i] - current_state.velocity[i]);
        control_output.effort[i] = 0;
      }
    }

    return control_output;
  }
}

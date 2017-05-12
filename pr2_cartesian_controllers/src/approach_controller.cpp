#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace cartesian_controllers {

  void ApproachController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::GuardedApproachGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::TwistStamped twist;
    Eigen::Vector3d approach_direction;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    if(goal->arm < 0 || goal->arm >= NUM_ARMS)
    {
      ROS_ERROR("Received a goal where the requested arm (%d) does not exist", goal->arm);
      action_server_->setAborted();
      return;
    }

    arm_index_ = goal->arm;
    is_contact_ = false;
    initial_contact_ = ros::Time(0);
    twist = goal->approach_command;
    tf::twistMsgToKDL(twist.twist, velocity_reference_);

    feedback_.velocity_reference.clear();
    for (int i = 0; i < 6; i++)
    {
      feedback_.velocity_reference.push_back(velocity_reference_(i));
    }

    force_threshold_ = goal->contact_force;
    approach_direction << velocity_reference_.vel.data[0], velocity_reference_.vel.data[1], velocity_reference_.vel.data[2];
    initial_force_ = measured_wrench_[arm_index_].block<3,1>(0,0).dot(approach_direction);
    loadParams();
    ROS_INFO("Approach controller server received a goal!");
  }

  void ApproachController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    ROS_WARN("Approach controller preempted!");
  }

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
          tf::wrenchEigenToMsg(measured_wrench_[arm_index_], feedback_.current_wrench.wrench);
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

  bool ApproachController::loadParams()
  {
    if (!getParam("/approach_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/approach_controller/rotational_gains", rot_gains_))
    {
      return false;
    }

    double t;
    if (!getParam("/approach_controller/contact_detection_time", t))
    {
      return false;
    }

    contact_detection_time_ = ros::Duration(t);

    if (rot_gains_.size() != 3)
    {
      ROS_ERROR("The rotational gains vector must have length 3! (Has length %zu)", rot_gains_.size());
      return false;
    }

    return true;
  }

  ApproachController::ApproachController() : ControllerTemplate<pr2_cartesian_controllers::GuardedApproachAction,
                                            pr2_cartesian_controllers::GuardedApproachFeedback,
                                            pr2_cartesian_controllers::GuardedApproachResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    feedback_thread_ = boost::thread(boost::bind(&ApproachController::publishFeedback, this));
  }

  ApproachController::~ApproachController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
  }

  sensor_msgs::JointState ApproachController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output = current_state;
    Eigen::Vector3d approach_direction;
    KDL::Frame current_pose;
    KDL::Twist twist_error;
    double alpha, beta, gamma;

    if (!action_server_->isActive())
    {
      return lastState(current_state);
    }


    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    if(!getChainJointState(current_state, chain_[arm_index_], joint_positions_[arm_index_], joint_velocities_[arm_index_]))
    {
      ROS_ERROR("Failed to get the chain joint state. Aborting.");
      action_server_->setAborted();
      lastState(current_state);
    }

    if (!has_initial_)
    {
      fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], initial_pose_);
      has_initial_ = true;
    }
    KDL::JntArray commanded_joint_velocities(chain_[arm_index_].getNrOfJoints());
    approach_direction << velocity_reference_.vel.data[0], velocity_reference_.vel.data[1], velocity_reference_.vel.data[2];
    approach_direction = approach_direction/approach_direction.norm();

    double approach_direction_force = std::abs(measured_wrench_[arm_index_].block<3,1>(0,0).dot(approach_direction) - initial_force_);
    feedback_.approach_direction_force = approach_direction_force;
    if (approach_direction_force > force_threshold_)
    {
      if (!is_contact_)
      {
        is_contact_ = true;
        initial_contact_ = ros::Time::now();
      }

      if ((ros::Time::now() - initial_contact_) > contact_detection_time_) // reject momentary force peaks
      {
        action_server_->setSucceeded(result_, "contact force achieved");
        has_initial_ = false;
        return current_state;
      }
    }
    else
    {
      is_contact_ = false;
    }

    fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], current_pose);
    twist_error = KDL::diff(current_pose, initial_pose_);

    // Update the commanded rotational velocities based on the
    // angular error between the current frame and the initial one.
    // We want to keep the same orientation from start to finish.
    // TODO: Make method
    velocity_reference_(3) = rot_gains_[0]*twist_error(3);
    velocity_reference_(4) = rot_gains_[1]*twist_error(4);
    velocity_reference_(5) = rot_gains_[2]*twist_error(5);
    feedback_.commanded_twist.header.stamp = ros::Time::now();
    feedback_.error_twist.header.stamp = ros::Time::now();
    tf::twistKDLToMsg(velocity_reference_, feedback_.commanded_twist.twist);
    tf::twistKDLToMsg(twist_error, feedback_.error_twist.twist);

    ikvel_[arm_index_]->CartToJnt(joint_positions_[arm_index_], velocity_reference_, commanded_joint_velocities);

    int joint_index;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        joint_index = getJointIndex(actuated_joint_names_[arm_index_], current_state.name[i]);
        control_output.velocity[i] = commanded_joint_velocities(joint_index);
        control_output.position[i] = current_state.position[i];
      }
    }

    return control_output;
  }
}

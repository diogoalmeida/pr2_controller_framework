#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace cartesian_controllers {

  void ApproachController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::GuardedApproachGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::TwistStamped twist;
    geometry_msgs::Vector3Stamped approach_direction_msg;

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
    approach_direction_msg.header = twist.header;
    approach_direction_msg.vector = twist.twist.linear;
    try
    {
      listener_.transformVector(base_link_, approach_direction_msg, approach_direction_msg);
      twist.header = approach_direction_msg.header;
      twist.twist.linear = approach_direction_msg.vector;
      tf::twistMsgToKDL(twist.twist, velocity_reference_);

      feedback_.velocity_reference.clear();
      for (int i = 0; i < 6; i++)
      {
        feedback_.velocity_reference.push_back(velocity_reference_(i));
      }

      force_threshold_ = goal->contact_force;
      approach_direction_ << velocity_reference_.vel.data[0], velocity_reference_.vel.data[1], velocity_reference_.vel.data[2];
      approach_direction_ = approach_direction_/approach_direction_.norm();
      initial_force_ = wrenchInFrame(arm_index_, base_link_).block<3,1>(0,0).dot(approach_direction_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
      action_server_->setAborted();
      return;
    }

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

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          feedback_.current_wrench.header.frame_id = base_link_;
          feedback_.current_wrench.header.stamp = ros::Time::now();
          tf::wrenchEigenToMsg(wrenchInFrame(arm_index_, base_link_), feedback_.current_wrench.wrench);
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
    std::vector<double> rot_gains;
    Eigen::Matrix<double, 6, 1> rot_gains_eig;

    if (!getParam("/approach_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/approach_controller/rotational_gains", rot_gains))
    {
      return false;
    }

    double t;
    if (!getParam("/approach_controller/contact_detection_time", t))
    {
      return false;
    }

    contact_detection_time_ = ros::Duration(t);

    if (rot_gains.size() != 6)
    {
      ROS_ERROR("The rotational gains vector must have length 6! (Has length %zu)", rot_gains.size());
      return false;
    }

    for (int i = 0; i < 6; i++)
    {
      rot_gains_eig[i] = rot_gains[i];
    }

    twist_controller_.reset(new TwistController(rot_gains_eig));

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
    KDL::Frame current_pose;
    KDL::Twist twist_comp;
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

    control_output = lastState(current_state, arm_index_);
    KDL::JntArray commanded_joint_velocities(chain_[arm_index_].getNrOfJoints());

    double approach_direction_force = std::abs(wrenchInFrame(arm_index_, base_link_).block<3,1>(0,0).dot(approach_direction_) - initial_force_);
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
        return control_output;
      }
    }
    else
    {
      is_contact_ = false;
    }

    fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], current_pose);

    // Update the commanded rotational velocities based on the
    // angular error between the current frame and the initial one.
    // We want to keep the same orientation from start to finish.
    twist_comp = twist_controller_->computeError(current_pose, initial_pose_);

    feedback_.commanded_twist.header.stamp = ros::Time::now();
    feedback_.error_twist.header.stamp = ros::Time::now();
    tf::twistKDLToMsg(velocity_reference_ + twist_comp, feedback_.commanded_twist.twist);
    tf::twistKDLToMsg(twist_comp, feedback_.error_twist.twist);
    ikvel_[arm_index_]->CartToJnt(joint_positions_[arm_index_], velocity_reference_ + twist_comp, commanded_joint_velocities);

    int joint_index = 0;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      control_output.velocity[i] = 0;

      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        control_output.velocity[i] = commanded_joint_velocities(joint_index);
        control_output.position[i] = current_state.position[i];
        joint_index++;
      }
    }

    return control_output;
  }
}

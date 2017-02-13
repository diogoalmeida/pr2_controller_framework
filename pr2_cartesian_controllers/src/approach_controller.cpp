#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace cartesian_controllers {

  void ApproachController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::GuardedApproachGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::TwistStamped twist;
    Eigen::Vector3d approach_direction;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

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
    initial_force_ = measured_wrench_.block<3,1>(0,0).dot(approach_direction);
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

  bool ApproachController::loadParams()
  {
    if (!nh_.getParam("/approach_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/approach_controller/action_server_name)");
      return false;
    }

    if (!nh_.getParam("/approach_controller/rotational_gains", rot_gains_))
    {
      ROS_ERROR("Missing vector with rotational gains (/approach_controller/rotational_gains)");
      return false;
    }

    double t;
    if (!nh_.getParam("/approach_controller/contact_detection_time", t))
    {
      ROS_ERROR("Missing contact_detection_time (/approach_controller/contact_detection_time)");
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
    delete action_server_;
  }

  sensor_msgs::JointState ApproachController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::JntArray commanded_joint_velocities(chain_.getNrOfJoints());
    Eigen::Vector3d approach_direction;
    KDL::Frame current_pose;
    KDL::Twist twist_error;
    double alpha, beta, gamma;

    if (!action_server_->isActive())
    {
      return lastState(current_state);
    }

    for (int i = 0; i < chain_.getNrOfJoints(); i++)
    {
      joint_positions_(i) = current_state.position[i];
    }

    if (!has_initial_)
    {
      fkpos_->JntToCart(joint_positions_, initial_pose_);
      has_initial_ = true;
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    approach_direction << velocity_reference_.vel.data[0], velocity_reference_.vel.data[1], velocity_reference_.vel.data[2];
    approach_direction = approach_direction/approach_direction.norm();

    double approach_direction_force = std::abs(measured_wrench_.block<3,1>(0,0).dot(approach_direction) - initial_force_);
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

    fkpos_->JntToCart(joint_positions_, current_pose);
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

    ikvel_->CartToJnt(joint_positions_, velocity_reference_, commanded_joint_velocities);

    control_output = current_state;

    for (int i = 0; i < chain_.getNrOfJoints(); i++)
    {
      control_output.velocity[i] = commanded_joint_velocities(i);
      control_output.position[i] = current_state.position[i];
    }

    return control_output;
  }
}

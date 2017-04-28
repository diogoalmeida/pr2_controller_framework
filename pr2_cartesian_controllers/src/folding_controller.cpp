#include <pr2_cartesian_controllers/folding_controller.hpp>

namespace cartesian_controllers {

  FoldingController::FoldingController() : ControllerTemplate<pr2_cartesian_controllers::FoldingControllerAction,
                                                pr2_cartesian_controllers::FoldingControllerFeedback,
                                                pr2_cartesian_controllers::FoldingControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    finished_acquiring_goal_ = false;
    feedback_thread_ = boost::thread(boost::bind(&FoldingController::publishFeedback, this));
  }

  FoldingController::~FoldingController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
  }

  void FoldingController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    ROS_WARN("Folding controller preempted!");
  }

  void FoldingController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::FoldingControllerGoal> goal = action_server_->acceptNewGoal();

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    ROS_INFO("Folding controller server received a goal!");
  }

  void FoldingController::publishFeedback()
  {
    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
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

  bool FoldingController::loadParams()
  {

    return true;
  }

  sensor_msgs::JointState FoldingController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output = current_state;

    if (!action_server_->isActive() || !finished_acquiring_goal_) // TODO: should be moved to parent class
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    if(!getChainJointState(current_state, chain_[surface_arm_], joint_positions_[surface_arm_], joint_velocities_[surface_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the surface arm. Aborting.");
      action_server_->setAborted();
      lastState(current_state);
    }

    if(!getChainJointState(current_state, chain_[rod_arm_], joint_positions_[rod_arm_], joint_velocities_[rod_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the rod arm. Aborting.");
      action_server_->setAborted();
      lastState(current_state);
    }

    KDL::Frame end_effector_kdl, grasp_point_kdl;

    // fkpos_->JntToCart(joint_positions_, end_effector_kdl);
    // fkvel_->JntToCart(joint_velocities_, end_effector_velocity_kdl);

    return control_output;
  }
}

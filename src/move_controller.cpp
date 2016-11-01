#include <pr2_cartesian_controllers/move_controller.hpp>

namespace manipulation {
  MoveController::MoveController()
  {
    nh_ = ros::NodeHandle("~");

    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    // Initialize KDL variables
    joint_positions_.resize(7);
    kdl_parser::treeFromUrdfModel(model_, tree_); // convert URDF description of the robot into a KDL tree
    tree_.getChain(base_link_, end_effector_link_, chain_);
    fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);
    ikpos_ = new KDL::ChainIkSolverPos_LMA(chain_);

    // Initialize actionlib server
    action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_controllers::MoveAction>(nh_, action_name_, false);

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&MoveController::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&MoveController::preemptCB, this));

    action_server_->start();
    ROS_INFO("Move controller initialized successfully!");
  }

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

    if (!nh_.getParam("/move_controller/end_effector_link_name", end_effector_link_))
    {
      ROS_ERROR("Missing end-effector link name (/move_controller/end_effector_link_name)");
      return false;
    }

    if (!nh_.getParam("/move_controller/base_link_name", base_link_))
    {
      ROS_ERROR("Missing base link name (/approach_controller/base_link_name)");
      return false;
    }

    if(!model_.initParam("/robot_description")){
        ROS_ERROR("ERROR getting robot description (/robot_description)");
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

#include <manipulation_controller/manipulation_controller.hpp>

namespace manipulation {
  ManipulationController::ManipulationController()
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
    ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);

    // Subscribe to force and torque measurements
    ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ManipulationController::forceTorqueCB, this);

    // Initialize actionlib server
    action_server_ = new actionlib::SimpleActionServer<manipulation_controller::ManipulationControllerAction>(nh_, action_name_, false);

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&ManipulationController::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&ManipulationController::preemptCB, this));

    action_server_->start();
  }

  /*
    Update current force and torque values.
  */
  void ManipulationController::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    geometry_msgs::Vector3Stamped vector_in, vector_out;
    geometry_msgs::Wrench transformed_wrench;
    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    vector_in.vector = msg->wrench.torque;
    vector_in.header = msg->header;
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.torque = vector_in.vector;

    vector_in.vector = msg->wrench.force;
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.force = vector_in.vector;

    tf::wrenchMsgToEigen(transformed_wrench, measured_wrench_);
  }

  /*
    Receive a new actiongoal: update controller input parameters.
  */
  void ManipulationController::goalCB()
  {
    boost::shared_ptr<const manipulation_controller::ManipulationControllerGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::PoseStamped pose_in, pose_out;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    pose_in = goal->surface_frame;
    listener_.transformPose(base_link_, pose_in, pose_out);
    tf::poseMsgToEigen(pose_out.pose, surface_frame_);

    pose_in = goal->goal_pose;
    listener_.transformPose(base_link_, pose_in, pose_out);
    tf::poseMsgToEigen(pose_out.pose, goal_pose_);
  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ManipulationController::publishFeedback()
  {

  }

  /*
    Search for controller relevant parameters in the parameter server
  */
  bool ManipulationController::loadParams()
  {
    if (!nh_.getParam("/manipulation_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/manipulation_controller/action_server_name)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/end_effector_link_name", end_effector_link_))
    {
      ROS_ERROR("Missing end-effector link name (/manipulation_controller/end_effector_link_name)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/base_link_name", base_link_))
    {
      ROS_ERROR("Missing base link name (/manipulation_controller/base_link_name)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/wdls_epsilon", eps_))
    {
      ROS_ERROR("Missing wdls epsilon (/manipulation_controller/wdls_epsilon)");
      return false;
    }

    if(!model_.initParam("/robot_description")){
        ROS_ERROR("ERROR getting robot description (/robot_description)");
        return false;
    }
  }

  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::Frame end_effector_kdl;
    Eigen::Vector3d rotation_axis, surface_tangent, surface_normal, force, torque;

    if (!action_server_->isActive())
    {
      return current_state;
    }

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    for (int i = 0; i < 7; i++)
    {
      joint_positions_(i) = current_state.position[i];
    }
    fkpos_->JntToCart(joint_positions_, end_effector_kdl);
    tf::transformKDLToEigen(end_effector_kdl, end_effector_pose_);

    // TODO: Get the proper vectors
    rotation_axis = end_effector_pose_.matrix().block<1,3>(0,2);
    surface_normal = surface_frame_.matrix().block<1,3>(0,2);
    surface_tangent = surface_frame_.matrix().block<1,3>(0,1);

    // Magic happens
    force = measured_wrench_.block<1,3>(0,0);
    torque = measured_wrench_.block<1,3>(3,0);
    estimated_orientation_ = torque.dot(rotation_axis)/k_spring_ + std::acos(surface_tangent.dot(end_effector_pose_.matrix().block<1,3>(0,1))); // TODO: Check indeces
    estimated_length_ = torque.dot(rotation_axis)/force.dot(surface_normal);
    estimated_r_ = (end_effector_pose_.matrix().block<1,3>(0,3).dot(surface_tangent) - estimated_length_*std::cos(estimated_orientation_))*surface_tangent;

    return control_output;
  }
}

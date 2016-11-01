#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace manipulation {
  ApproachController::ApproachController()
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
    ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ApproachController::forceTorqueCB, this);

    // Initialize actionlib server
    action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_controllers::GuardedApproachAction>(nh_, action_name_, false);

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&ApproachController::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&ApproachController::preemptCB, this));

    action_server_->start();

    boost::thread(boost::bind(&ApproachController::publishFeedback, this));
  }

  /*
    Update current force and torque values.
  */
  void ApproachController::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
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
  void ApproachController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::GuardedApproachGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::TwistStamped twist;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    twist = goal->approach_command;
    tf::twistMsgToKDL(twist.twist, velocity_reference_);

    force_threshold_ = goal->contact_force;
  }

  /*
    Preempt controller.
  */
  void ApproachController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ApproachController::publishFeedback()
  {
    ros::Rate feedback_rate(feedback_hz_);
    feedback_.current_wrench.header.frame_id = base_link_;

    while(ros::ok())
    {
      if (action_server_->isActive())
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        feedback_.current_wrench.header.stamp = ros::Time::now();
        tf::wrenchEigenToMsg(measured_wrench_, feedback_.current_wrench.wrench);
        action_server_->publishFeedback(feedback_);
      }

      feedback_rate.sleep();
    }
  }

  /*
    Search for controller relevant parameters in the parameter server
  */
  bool ApproachController::loadParams()
  {
    if (!nh_.getParam("/approach_controller/action_server_name", action_name_))
    {
      ROS_ERROR("Missing action server name parameter (/approach_controller/action_server_name)");
      return false;
    }

    if (!nh_.getParam("/approach_controller/end_effector_link_name", end_effector_link_))
    {
      ROS_ERROR("Missing end-effector link name (/approach_controller/end_effector_link_name)");
      return false;
    }

    if (!nh_.getParam("/approach_controller/base_link_name", base_link_))
    {
      ROS_ERROR("Missing base link name (/approach_controller/base_link_name)");
      return false;
    }

    if (!nh_.getParam("/approach_controller/wdls_epsilon", eps_))
    {
      ROS_ERROR("Missing wdls epsilon (/approach_controller/wdls_epsilon)");
      return false;
    }

    if (!nh_.getParam("/approach_controller/feedback_rate", feedback_hz_))
    {
      ROS_ERROR("Missing feedback_rate (/approach_controller/feedback_rate)");
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
  sensor_msgs::JointState ApproachController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::JntArray commanded_joint_velocities;

    if (!action_server_->isActive())
    {
      return current_state;
    }

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    if (measured_wrench_.block<3,1>(0,0).norm() > force_threshold_)
    {
      action_server_->setSucceeded(result_, "contact force achieved");
      return current_state;
    }

    for (int i = 0; i < 7; i++)
    {
      joint_positions_(i) = current_state.position[i];
    }

    ikvel_->CartToJnt(joint_positions_, velocity_reference_, commanded_joint_velocities);

    control_output = current_state;

    for (int i = 0; i < 7; i++)
    {
      control_output.velocity[i] = commanded_joint_velocities(i);
    }

    return control_output;
  }
}

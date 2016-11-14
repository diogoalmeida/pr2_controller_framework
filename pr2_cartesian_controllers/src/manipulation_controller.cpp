#include <pr2_cartesian_controllers/manipulation_controller.hpp>

namespace cartesian_controllers {
  /*
    Preempt controller.
  */
  void ManipulationController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
  }

  /*
    Receive a new actiongoal: update controller input parameters.
  */
  void ManipulationController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::ManipulationControllerGoal> goal = action_server_->acceptNewGoal();
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
    ros::Rate feedback_rate(feedback_hz_);
    visualization_msgs::Marker object_pose;
    std_msgs::ColorRGBA object_color;
    Eigen::Vector3d r_1;

    object_color.r = 1;
    object_color.g = 0;
    object_color.b = 0;
    object_color.a = 1;

    object_pose.ns = "manipulation_controller";
    object_pose.id = 1;
    object_pose.type = object_pose.CYLINDER;
    object_pose.action = object_pose.ADD;
    object_pose.color = object_color;
    object_pose.lifetime = ros::Duration(0);
    object_pose.frame_locked = false; // not sure about this

    while(ros::ok())
    {
      if (action_server_->isActive())
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        r_1 = estimated_r_/estimated_r_.norm();
        tf::poseEigenToMsg(end_effector_pose_, object_pose.pose);
        // TODO: Needs to be checked
        object_pose.scale.x = estimated_length_;
        object_pose.scale.y = 0.02;
        object_pose.scale.z = 0.02;

        feedback_.object_pose = object_pose;
        action_server_->publishFeedback(feedback_);
      }
      feedback_rate.sleep();
    }
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

    if (!nh_.getParam("manipulation_controller/spring_constant", k_spring_))
    {
      ROS_ERROR("Missing spring constant (/manipulation_controller/spring_constant)");
      return false;
    }

    double k_1, k_2, k_3;
    if (!nh_.getParam("manipulation_controller/gains/k_1", k_1))
    {
      ROS_ERROR("Missing k_1 (/manipulation_controller/gains/k_1)");
      return false;
    }

    if (!nh_.getParam("manipulation_controller/gains/k_2", k_2))
    {
      ROS_ERROR("Missing k_2 (/manipulation_controller/gains/k_2)");
      return false;
    }

    if (!nh_.getParam("manipulation_controller/gains/k_3", k_3))
    {
      ROS_ERROR("Missing k_3 (/manipulation_controller/gains/k_3)");
      return false;
    }

    control_gains_ << k_1, 0  , 0  ,
                      0  , k_2, 0  ,
                      0  , 0  , k_3;
    return true;
  }

  /*
    Estimates the pose of the grasped object with respect to the end-effector
  */
  void ManipulationController::estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt)
  {
    Eigen::Vector3d force, torque;

    // Estimate the grasped object pose. Current: direct computation
    force = measured_wrench_.block<1,3>(0,0);
    torque = measured_wrench_.block<1,3>(3,0);
    estimated_orientation_ = torque.dot(rotation_axis)/k_spring_ + std::acos(surface_tangent.dot(end_effector_pose_.matrix().block<1,3>(0,1))); // TODO: Check indeces
    estimated_length_ = torque.dot(rotation_axis)/force.dot(surface_normal);
    estimated_r_ = (end_effector_pose_.matrix().block<1,3>(0,3).dot(surface_tangent) - estimated_length_*std::cos(estimated_orientation_))*surface_tangent;
  }

  /*
    Implements the control strategy. This method is expected to call at a rate of approximately 1000 Hz. It should never
    take more than 1ms to execute.
  */
  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::Frame end_effector_kdl;
    KDL::JntArray commanded_joint_velocities;
    KDL::Twist input_twist;
    Eigen::Vector3d rotation_axis, surface_tangent, surface_normal, force, torque, errors, commands;
    double x_e, y_e, theta_e, x_d, y_d, theta_d;
    Eigen::Matrix3d inv_g;
    Eigen::Matrix<double, 6, 1> twist_eig;

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
    fkpos_->JntToCart(joint_positions_, end_effector_kdl);
    tf::transformKDLToEigen(end_effector_kdl, end_effector_pose_);


    // TODO: Get the proper vectors
    rotation_axis = end_effector_pose_.matrix().block<1,3>(0,2);
    surface_normal = surface_frame_.matrix().block<1,3>(0,2);
    surface_tangent = surface_frame_.matrix().block<1,3>(0,1);

    estimatePose(rotation_axis, surface_tangent, surface_normal, dt);

    // Compute the cartesian twist to command the end-effector. Current: straightly compensate for the kinematics
    inv_g << 1, -estimated_length_*std::sin(estimated_orientation_), 0                           ,
             0, -estimated_length_*std::cos(estimated_orientation_), 0                           ,
             0, -1                                                 , -estimated_length_/k_spring_;


    Eigen::AngleAxisd goal_aa(goal_pose_.rotation()), end_effector_aa(end_effector_pose_.rotation());

    x_d = goal_pose_.matrix().block<1,3>(0,3).dot(surface_tangent);
    y_d = goal_pose_.matrix().block<1,3>(0,3).dot(surface_normal);
    theta_d = goal_aa.angle();

    x_e = end_effector_pose_.matrix().block<1,3>(0,3).dot(surface_tangent);
    y_e = end_effector_pose_.matrix().block<1,3>(0,3).dot(surface_normal);
    theta_d = end_effector_aa.angle();

    errors <<  x_d - x_e,
               y_d - y_e,
               theta_d - theta_e;

    commands = control_gains_*inv_g*errors;
    twist_eig << commands[0]*surface_tangent + commands[1]*surface_normal, commands[2]*end_effector_aa.axis();

    tf::twistEigenToKDL(twist_eig, input_twist);

    ikvel_->CartToJnt(joint_positions_, input_twist, commanded_joint_velocities);

    control_output = current_state;

    for (int i = 0; i < 7; i++)
    {
      control_output.velocity[i] = commanded_joint_velocities(i);
    }

    return control_output;
  }
}

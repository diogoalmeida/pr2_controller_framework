#include <pr2_cartesian_controllers/manipulation_controller.hpp>

namespace cartesian_controllers {
  /*
    Preempt controller.
  */
  void ManipulationController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    ROS_WARN("Manipulation controller preempted!");
  }

  /*
    Receive a new actiongoal: update controller input parameters.
    Generates:

    - rot_gains_ in the base frame
    - surface_frame_ in the base frame
    - goal_pose_ in the base frame
    - end_effector_to_grasp_point_ in the end effector link frame
  */
  void ManipulationController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::ManipulationControllerGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::PoseStamped pose_in, pose_out;
    std::string surface_frame_name;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    debug_twist_ = false;
    use_debug_eef_to_grasp_ = false;
    surface_rotation_axis_ = false;

    if (goal->is_debug)
    {
      debug_twist_ = true;
    }

    if (goal->use_debug_eef_to_grasp)
    {
      use_debug_eef_to_grasp_ = true;
    }

    if (goal->use_surface_rotation_axis)
    {
      surface_rotation_axis_ = true;
    }

    force_d_ = goal->desired_contact_force;

    if (!loadParams())
    {
      action_server_->setAborted();
    }

    pose_in = goal->surface_frame;
    rot_gains_.header.frame_id = pose_in.header.frame_id;
    listener_.waitForTransform(ft_frame_id_, base_link_, ros::Time(0), ros::Duration(0.1));
    try
    {
      // get surface frame
      rot_gains_.header.stamp = ros::Time(0);
      listener_.transformVector(base_link_, rot_gains_, rot_gains_);
      pose_in.header.stamp = ros::Time(0);
      listener_.transformPose(base_link_, pose_in, pose_out);
      tf::poseMsgToEigen(pose_out.pose, surface_frame_);

      // get the goal pose
      pose_in = goal->goal_pose;
      listener_.transformPose(base_link_, pose_in, pose_out);
      tf::poseMsgToEigen(pose_out.pose, goal_pose_);

      tf::vectorMsgToEigen(goal->debug_eef_to_grasp, debug_eef_to_grasp_eig_);
      debug_x_ = goal->debug_twist.linear.x;
      debug_y_ = goal->debug_twist.linear.y;
      debug_rot_ = goal->debug_twist.angular.z;

      // get the relationship between kinematic chain end-effector and
      // tool-tip (grasping point)
      pose_in.header.frame_id = ft_frame_id_;
      pose_in.header.stamp = ros::Time(0); // get latest available

      pose_in.pose.position.x = 0;
      pose_in.pose.position.y = 0;
      pose_in.pose.position.z = 0;

      pose_in.pose.orientation.x = 0;
      pose_in.pose.orientation.y = 0;
      pose_in.pose.orientation.z = 0;
      pose_in.pose.orientation.w = 1;
      listener_.transformPose(end_effector_link_, pose_in, pose_out);
      tf::poseMsgToKDL(pose_out.pose, end_effector_to_grasp_point_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
      action_server_->setAborted();
    }

    ROS_INFO("Manipulation controller server received a goal!");
  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ManipulationController::publishFeedback()
  {
    visualization_msgs::Marker object_pose, desired_pose, eef_to_grasp_marker;
    std_msgs::ColorRGBA object_color;
    geometry_msgs::Pose grasp_pose_geo;
    Eigen::Vector3d r_1, goal_r;
    tf::Transform transform;

    object_color.r = 1;
    object_color.g = 0;
    object_color.b = 0;
    object_color.a = 1;

    object_pose.ns = "manipulation_controller";
    object_pose.id = 1;
    object_pose.type = object_pose.LINE_STRIP;
    object_pose.action = object_pose.ADD;
    object_pose.color = object_color;
    object_pose.lifetime = ros::Duration(0);
    object_pose.frame_locked = true; // not sure about this
    object_pose.pose.orientation.w = 1;
    object_pose.header.frame_id = base_link_;
    object_pose.scale.x = 0.02;
    desired_pose = object_pose;
    desired_pose.id = 2;
    desired_pose.color.r = 0;
    desired_pose.color.g = 1;
    eef_to_grasp_marker = desired_pose;
    eef_to_grasp_marker.id = 3;
    eef_to_grasp_marker.type = eef_to_grasp_marker.ARROW;
    eef_to_grasp_marker.scale.y = 0.01;
    eef_to_grasp_marker.scale.z = 0.01;

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          r_1 = estimated_r_;

          getMarkerPoints(goal_pose_.translation(), goal_pose_.translation() + hardcoded_length_*goal_pose_.rotation().block<3,1>(0,0), desired_pose);
          getMarkerPoints(grasp_point_pose_.translation(), grasp_point_pose_.translation() + r_1, object_pose);

          object_pose.header.stamp = ros::Time::now();

          tf::poseEigenToMsg(grasp_point_pose_, grasp_pose_geo);
          transform.setOrigin(tf::Vector3(grasp_pose_geo.position.x, grasp_pose_geo.position.y, grasp_pose_geo.position.z));
          transform.setRotation(tf::Quaternion (grasp_pose_geo.orientation.x, grasp_pose_geo.orientation.y, grasp_pose_geo.orientation.z, grasp_pose_geo.orientation.w));
          broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_, "computed_grasp_point"));

          getMarkerPoints(end_effector_pose_.translation(), grasp_point_pose_.translation(), eef_to_grasp_marker);

          current_pub_.publish(object_pose);
          target_pub_.publish(desired_pose);
          eef_to_grasp_pub_.publish(eef_to_grasp_marker);

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

  /*
    Fills a marker with the given initial and end point. Clears existing points.
  */
  void ManipulationController::getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker)
  {
    geometry_msgs::Point point;

    marker.points.clear();
    tf::pointEigenToMsg(initial_point, point);
    marker.points.push_back(point);
    tf::pointEigenToMsg(final_point, point);
    marker.points.push_back(point);
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

    if (!nh_.getParam("/manipulation_controller/spring_constant", k_spring_))
    {
      ROS_ERROR("Missing spring constant (/manipulation_controller/spring_constant)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/estimate_length", estimate_length_))
    {
      ROS_ERROR("Missing estimate length (/manipulation_controller/estimate_length)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/hardcoded_length", hardcoded_length_))
    {
      ROS_ERROR("Missing hardcoded length (/manipulation_controller/hardcoded_length)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/initial_angle_offset", initial_angle_offset_))
    {
      ROS_ERROR("Missing initial angle offset (/manipulation_controller/initial_angle_offset)");
      return false;
    }

    double k_1, k_2, k_3;
    if (!nh_.getParam("/manipulation_controller/gains/k_1", k_1))
    {
      ROS_ERROR("Missing k_1 (/manipulation_controller/gains/k_1)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/gains/k_2", k_2))
    {
      ROS_ERROR("Missing k_2 (/manipulation_controller/gains/k_2)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/gains/k_3", k_3))
    {
      ROS_ERROR("Missing k_3 (/manipulation_controller/gains/k_3)");
      return false;
    }

    control_gains_ << k_1, 0  , 0  ,
                      0  , k_2, 0  ,
                      0  , 0  , k_3;

    std::vector<double> rot_gains;
    if (!nh_.getParam("/manipulation_controller/rotational_gains", rot_gains))
    {
      ROS_ERROR("Missing vector with rotational gains (/manipulation_controller/rotational_gains)");
      return false;
    }

    if (rot_gains.size() != 3)
    {
      ROS_ERROR("The rotational gains vector must have length 3! (Has length %zu)", rot_gains.size());
      return false;
    }

    // convert to geometry msg. These gains are assumed to represent gains along
    // the surface frame (which is the frame of the goal pose) x, y and z axis, respectively
    rot_gains_.header.frame_id = base_link_; // these gains are expressed in the surface frame, which is given upon receiving a goal
    rot_gains_.header.stamp = ros::Time::now();
    rot_gains_.vector.x = rot_gains[0];
    rot_gains_.vector.y = rot_gains[1];
    rot_gains_.vector.z = rot_gains[2];

    return true;
  }

  /*
    Estimates the pose of the grasped object with respect to the end-effector.
    Assumes rotation axis, surface tangent and normal in the base frame.
  */
  void ManipulationController::estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt)
  {
    Eigen::Vector3d force, torque, r_dir;
    double spring_deflection, end_effector_angle;

    // Estimate the grasped object pose. Current: direct computation
    // The force and torque data are assumed to be exerted in the grasping
    // point
    force = measured_wrench_.block<3,1>(0,0);
    torque = measured_wrench_.block<3,1>(3,0);

    // estimated orientation = end_effector_angle in the plane - estimated_spring deflection
    // end effector rotation axis is the y axis. TODO: parametrize
    // estimated spring deflection is given by the torque over the spring constant. TODO: apply kalman
    spring_deflection = -torque[1]/k_spring_ + initial_angle_offset_; // offset w.r.t 0 angle in the end-effector axis TODO: Fix this
    feedback_.spring_angle = spring_deflection;
    end_effector_angle = std::acos(surface_tangent.dot(grasp_point_pose_.matrix().block<3,1>(0,0)));
    feedback_.eef_angle = end_effector_angle;
    estimated_orientation_ = end_effector_angle - spring_deflection;
    r_dir = -(std::cos(estimated_orientation_)*surface_tangent + std::sin(estimated_orientation_)*surface_normal); // vector pointing from the grasping to the contact points

    if (!estimate_length_)
    {
      estimated_r_ = hardcoded_length_*r_dir;
    }
    else
    {
      estimated_length_ = std::abs(torque.norm()/force.norm());
      estimated_r_ = estimated_length_*r_dir;
    }
  }

  /*
    Implements the control strategy. This method is expected to call at a rate of approximately 1000 Hz. It should never
    take more than 1ms to execute.
  */
  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::Frame end_effector_kdl, grasp_point_kdl;
    KDL::JntArray commanded_joint_velocities(chain_.getNrOfJoints());
    KDL::Twist input_twist, twist_error;
    Eigen::Vector3d rotation_axis, surface_tangent, surface_normal, force, torque, errors, commands, origin, eef_to_grasp_eig, velocity_command, velocity_eef;
    double x_e, y_e, theta_e, x_d, y_d, theta_d, torque_c, force_c, x_c, theta_c;
    Eigen::Matrix3d inv_g, skew;
    Eigen::Matrix<double, 6, 1> twist_eig;

    if (!action_server_->isActive()) // TODO: should be moved to parent class
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    for (int i = 0; i < chain_.getNrOfJoints(); i++)
    {
      joint_positions_(i) = current_state.position[i];
    }

    if (!has_initial_)
    {
      fkpos_->JntToCart(joint_positions_, initial_pose_); // base_link
      has_initial_ = true;
    }

    fkpos_->JntToCart(joint_positions_, end_effector_kdl);
    grasp_point_kdl = end_effector_kdl*end_effector_to_grasp_point_;
    tf::transformKDLToEigen(end_effector_kdl, end_effector_pose_); // base_link
    tf::transformKDLToEigen(grasp_point_kdl, grasp_point_pose_); // base_link

    surface_normal = surface_frame_.matrix().block<3,1>(0,2); // base_link
    surface_tangent = surface_frame_.matrix().block<3,1>(0,0); // base_link
    origin = surface_frame_.matrix().block<3,1>(0,3); // base_link

    if (surface_rotation_axis_)
    {
      rotation_axis = -surface_frame_.matrix().block<3,1>(0,1); // base_link
    }
    else
    {
      rotation_axis = -grasp_point_pose_.matrix().block<3,1>(0,1); // base_link
    }

    estimatePose(rotation_axis, surface_tangent, surface_normal, dt);

    // Compute the cartesian twist to command the end-effector. Current: straightly compensate for the kinematics
    if (!estimate_length_)
    {
      inv_g = computeInvG(hardcoded_length_, estimated_orientation_);
    }
    else
    {
      inv_g = computeInvG(estimated_length_, estimated_orientation_);
    }

    Eigen::Vector3d goal_r = goal_pose_.matrix().block<3,1>(0,3) - origin; // vector from the surface frame (origin) to the goal pose (along the surface tangent)

    x_d = goal_r.dot(surface_tangent);
    theta_d = std::acos(surface_tangent.dot(goal_r));

    x_c = (grasp_point_pose_.matrix().block<3,1>(0,3) + estimated_r_).dot(surface_tangent);
    theta_c = estimated_orientation_;
    torque_c = measured_wrench_.block<3,1>(3,0)[1];
    force_c = measured_wrench_.block<3,1>(0,0).dot(surface_normal); // TODO: Fix this

    errors <<  x_d      -     x_c,
               theta_d  - theta_c,
               force_d_ - force_c;

    if (debug_twist_)
    {
      commands << debug_x_, debug_y_, debug_rot_;
    }
    else
    {
      commands = inv_g*control_gains_*errors; // this is the desired twist for the grasping point, in the 2d plane (\dot{x}, \dot{y}, \dot{\theta})
    }

    feedback_.x_c = x_c;
    feedback_.theta_c = theta_c;
    feedback_.f_c = force_c;
    feedback_.torque_c = torque_c;
    feedback_.f_e.x = measured_wrench_[0];
    feedback_.f_e.y = measured_wrench_[1];
    feedback_.f_e.z = measured_wrench_[2];
    feedback_.tau_e.x = measured_wrench_[3];
    feedback_.tau_e.y = measured_wrench_[4];
    feedback_.tau_e.z = measured_wrench_[5];
    feedback_.error_x = errors[0];
    feedback_.error_theta = errors[1];
    feedback_.error_force = errors[2];
    feedback_.commanded_x = commands[0];
    feedback_.commanded_y = commands[1];
    feedback_.commanded_rot = commands[2];

    velocity_command = commands[0]*surface_tangent + commands[1]*surface_normal;
    skew = computeSkewSymmetric(commands[2]*rotation_axis);

    if (debug_twist_ && use_debug_eef_to_grasp_)
    {
      velocity_eef = -skew*debug_eef_to_grasp_eig_ + velocity_command;
    }
    else
    {
      eef_to_grasp_eig = grasp_point_pose_.translation() - end_effector_pose_.translation();
      velocity_eef = -skew*eef_to_grasp_eig + velocity_command;
    }

    twist_eig << velocity_eef, commands[2]*rotation_axis; // convert input twist to the end-effector of the kinematic chain
    tf::twistEigenToKDL(twist_eig, input_twist);
    tf::twistEigenToMsg(twist_eig, feedback_.commanded_twist);

    twist_error = KDL::diff(end_effector_kdl, initial_pose_); // to maintain the movement on the initial planar direction TODO: Implement this properly

    input_twist(3) += rot_gains_.vector.x*twist_error(3);
    input_twist(4) += rot_gains_.vector.y*twist_error(4);
    input_twist(5) += rot_gains_.vector.z*twist_error(5);

    ikvel_->CartToJnt(joint_positions_, input_twist, commanded_joint_velocities);
    control_output = current_state;

    for (int i = 0; i < chain_.getNrOfJoints(); i++)
    {
      control_output.position[i] = joint_positions_(i) + commanded_joint_velocities(i)*dt.toSec();
      control_output.velocity[i] = commanded_joint_velocities(i);
    }

    return control_output;
  }

  /*
    Computes the inverse of the manipulation model matrix
  */
  Eigen::Matrix3d ManipulationController::computeInvG(double length, double angle)
  {
    Eigen::Matrix3d ret_val;

    ret_val << 1, -length*std::sin(angle),                0,
               0,  length*std::cos(angle),                0,
               0,      -1                ,      length/k_spring_;

    return ret_val;
  }

  /*
    Computes the skew-symmetric matrix of the provided vector
  */
  Eigen::Matrix3d ManipulationController::computeSkewSymmetric(const Eigen::Vector3d &v)
  {
    Eigen::Matrix3d S;

    S << 0,    -v(2),  v(1),
         v(2),  0   , -v(0),
        -v(1),  v(0),  0;

    return S;
  }
}

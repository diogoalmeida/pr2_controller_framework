#include <pr2_cartesian_controllers/manipulation_controller.hpp>

namespace cartesian_controllers {

  ManipulationController::ManipulationController() : ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                pr2_cartesian_controllers::ManipulationControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    target_pub_ = nh_.advertise<visualization_msgs::Marker>("manipulation_controller_target", 1);
    current_pub_ = nh_.advertise<visualization_msgs::Marker>("manipulation_controller_estimated", 1);
    ground_truth_pub_ = nh_.advertise<visualization_msgs::Marker>("manipulation_controller_ground_truth", 1);
    eef_to_grasp_pub_ = nh_.advertise<visualization_msgs::Marker>("eef_to_grasp_pose", 1);
    finished_acquiring_goal_ = false;
    feedback_thread_ = boost::thread(boost::bind(&ManipulationController::publishFeedback, this));
  }

  ManipulationController::~ManipulationController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
    delete action_server_;
  }

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
    - end_effector_to_grasp_point_ in the end effector link frame
  */
  void ManipulationController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::ManipulationControllerGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::PoseStamped pose_in, pose_out;
    Eigen::Vector3d init_x;
    double x_d, theta_d, f_d;

    finished_acquiring_goal_ = false;
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

    f_d = goal->desired_contact_force;
    x_d = goal->x_d;
    theta_d = goal->theta_d;

    x_d_ << x_d,
            theta_d,
            f_d;

    if (!loadParams())
    {
      action_server_->setAborted();
    }

    if (!ekf_estimator_.getParams(nh_))
    {
      action_server_->setAborted();
    }

    if (!controller_.getParams(nh_))
    {
      action_server_->setAborted();
    }

    pose_in = goal->surface_frame;
    rot_gains_.header.frame_id = pose_in.header.frame_id;
    listener_.waitForTransform(pose_in.header.frame_id, base_link_, ros::Time(0), ros::Duration(wait_for_tf_time_));
    try
    {
      // get surface frame
      rot_gains_.header.stamp = ros::Time(0);
      listener_.transformVector(base_link_, rot_gains_, rot_gains_);
      pose_in.header.stamp = ros::Time(0);
      listener_.transformPose(base_link_, pose_in, pose_out);
      tf::poseMsgToEigen(pose_out.pose, surface_frame_);

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

    finished_acquiring_goal_ = true;
    ROS_INFO("Manipulation controller server received a goal!");
  }

  /*
    Asynchronously publish a feedback message on the control status
  */
  void ManipulationController::publishFeedback()
  {
    visualization_msgs::Marker object_pose, desired_pose, ground_truth, eef_to_grasp_marker;
    std_msgs::ColorRGBA object_color;
    geometry_msgs::Pose grasp_pose_geo;
    tf::Transform transform;
    Eigen::Vector3d r_1, r_d, x_d_eigen, x_c_eigen, r_real, x_real_eigen;
    double estimated_length;

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
    ground_truth = object_pose;
    ground_truth.id = 3;
    ground_truth.color.g = 0;
    ground_truth.color.b = 1;
    eef_to_grasp_marker = desired_pose;
    eef_to_grasp_marker.id = 4;
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
          x_d_eigen = surface_frame_.translation() + x_d_[0]*surface_frame_.rotation().block<3,1>(0,0);
          x_c_eigen = surface_frame_.translation() + x_hat_[0]*surface_frame_.rotation().block<3,1>(0,0);
          x_real_eigen = surface_frame_.translation() + feedback_.x_c_2*surface_frame_.rotation().block<3,1>(0,0);
          r_d = cos(x_d_[1])*surface_frame_.rotation().block<3,1>(0,0) + sin(x_d_[1])*surface_frame_.rotation().block<3,1>(0,2); // r = cos(theta)*x + sin(theta)*z
          r_1 = cos(x_hat_[1])*surface_frame_.rotation().block<3,1>(0,0) + sin(x_hat_[1])*surface_frame_.rotation().block<3,1>(0,2);
          r_real = cos(feedback_.theta_c_2)*surface_frame_.rotation().block<3,1>(0,0) + sin(feedback_.theta_c_2)*surface_frame_.rotation().block<3,1>(0,2);
          estimated_length = (x_e_[0] - x_hat_[0])/cos(x_hat_[1]);

          getMarkerPoints(x_d_eigen, x_d_eigen + hardcoded_length_*r_d, desired_pose);
          getMarkerPoints(x_c_eigen, x_c_eigen + estimated_length*r_1, object_pose);
          getMarkerPoints(x_real_eigen, x_real_eigen + hardcoded_length_*r_real, ground_truth);

          object_pose.header.stamp = ros::Time::now();
          desired_pose.header.stamp = ros::Time::now();
          ground_truth.header.stamp = ros::Time::now();

          tf::poseEigenToMsg(grasp_point_pose_, grasp_pose_geo);
          transform.setOrigin(tf::Vector3(grasp_pose_geo.position.x, grasp_pose_geo.position.y, grasp_pose_geo.position.z));
          transform.setRotation(tf::Quaternion (grasp_pose_geo.orientation.x, grasp_pose_geo.orientation.y, grasp_pose_geo.orientation.z, grasp_pose_geo.orientation.w));
          // broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_, "computed_grasp_point"));

          getMarkerPoints(end_effector_pose_.translation(), grasp_point_pose_.translation(), eef_to_grasp_marker);

          current_pub_.publish(object_pose);
          target_pub_.publish(desired_pose);
          ground_truth_pub_.publish(ground_truth);
          // eef_to_grasp_pub_.publish(eef_to_grasp_marker);

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

    std::vector<double> rot_gains;
    if (!nh_.getParam("/manipulation_controller/rotational_gains", rot_gains))
    {
      ROS_ERROR("Missing vector with rotational gains (/manipulation_controller/rotational_gains)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/wait_for_tf_time", wait_for_tf_time_))
    {
      ROS_ERROR("Missing wait for tf time (/manipulation_controller/wait_for_tf_time)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/spring_constant", k_s_))
    {
      ROS_ERROR("Missing spring constant (/manipulation_controller/spring_constant)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/estimator/initial_x_offset", init_x_offset_))
    {
      ROS_ERROR("Missing initial_x_offset (/manipulation_controller/estimator/initial_x_offset)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/estimator/initial_theta_offset", init_theta_offset_))
    {
      ROS_ERROR("Missing initial_theta_offset (/manipulation_controller/estimator/initial_theta_offset)");
      return false;
    }

    if (!nh_.getParam("/manipulation_controller/initial_angle_offset", theta_o_))
    {
      ROS_ERROR("Missing initial angle offset (/manipulation_controller/initial_angle_offset)");
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
    Implements the control strategy. This method is expected to call at a rate of approximately 1000 Hz. It should never
    take more than 1ms to execute.
  */
  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::Frame end_effector_kdl, grasp_point_kdl, surface_frame_to_grasp, surface_frame_kdl;
    KDL::FrameVel end_effector_velocity_kdl, grasp_point_velocity_kdl;
    KDL::JntArray commanded_joint_velocities(chain_.getNrOfJoints());
    KDL::Twist input_twist, twist_error, actual_twist;
    Eigen::Affine3d surface_frame_to_grasp_eig;
    Eigen::Vector3d rotation_axis, surface_tangent, surface_normal, force, torque, commands, origin, eef_to_grasp_eig, velocity_command, velocity_eef, actual_commands;
    Eigen::Vector3d u, e, y, temp, surface_tangent_in_grasp, surface_normal_in_grasp;
    double torque_e, force_e, x_c, theta_c;
    Eigen::Matrix3d inv_g, skew;
    Eigen::Matrix<double, 6, 1> twist_eig, actual_twist_eigen;

    if (!action_server_->isActive() || !finished_acquiring_goal_) // TODO: should be moved to parent class
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    for (int i = 0; i < chain_.getNrOfJoints(); i++)
    {
      joint_positions_(i) = current_state.position[i];
      joint_velocities_.q(i) = current_state.position[i];
      joint_velocities_.qdot(i) = current_state.velocity[i];
    }

    fkpos_->JntToCart(joint_positions_, end_effector_kdl);
    fkvel_->JntToCart(joint_velocities_, end_effector_velocity_kdl);
    grasp_point_velocity_kdl = end_effector_velocity_kdl*end_effector_to_grasp_point_;
    grasp_point_kdl = end_effector_kdl*end_effector_to_grasp_point_;
    tf::transformEigenToKDL(surface_frame_, surface_frame_kdl);
    surface_frame_to_grasp = grasp_point_kdl.Inverse()*surface_frame_kdl;
    tf::transformKDLToEigen(surface_frame_to_grasp, surface_frame_to_grasp_eig);
    tf::transformKDLToEigen(end_effector_kdl, end_effector_pose_); // base_link
    tf::transformKDLToEigen(grasp_point_kdl, grasp_point_pose_); // base_link

    surface_normal = surface_frame_.matrix().block<3,1>(0,2); // base_link
    surface_tangent = surface_frame_.matrix().block<3,1>(0,0); // base_link
    surface_normal_in_grasp = surface_frame_to_grasp_eig.matrix().block<3,1>(0,2);
    surface_tangent_in_grasp = surface_frame_to_grasp_eig.matrix().block<3,1>(0,0);
    origin = surface_frame_.matrix().block<3,1>(0,3); // base_link

    x_e_ << (grasp_point_pose_.translation() - origin).dot(surface_tangent),
           (grasp_point_pose_.translation() - origin).dot(surface_normal),
           std::acos(surface_tangent.dot(-grasp_point_pose_.matrix().block<3,1>(0,0)));

    if (!has_initial_)
    {
      fkpos_->JntToCart(joint_positions_, initial_pose_); // base_link
      x_hat_[0] = x_e_[0] + init_x_offset_; // initial x_c estimate, made different from x_e_ to avoid dx = 0
      x_hat_[1] = x_e_[1] + init_theta_offset_;
      x_hat_[2] = measured_wrench_.block<3,1>(0,0).dot(surface_normal_in_grasp);
      ekf_estimator_.initialize(x_hat_);
      has_initial_ = true;
    }

    if (surface_rotation_axis_)
    {
      rotation_axis = -surface_frame_.matrix().block<3,1>(0,1); // base_link
    }
    else
    {
      rotation_axis = -grasp_point_pose_.matrix().block<3,1>(0,1); // base_link
    }

    torque_e = measured_wrench_.block<3,1>(3,0).dot(computeSkewSymmetric(surface_normal_in_grasp)*surface_tangent_in_grasp);
    force_e = measured_wrench_.block<3,1>(0,0).dot(surface_normal_in_grasp);

    if (debug_twist_)
    {
      commands << debug_x_, debug_y_, debug_rot_;
    }
    else
    {
      commands = controller_.compute(x_d_, x_hat_, x_e_);
    }

    // compute the measurements vector
    y << torque_e/force_e,
         force_e,
         x_e_[2] + torque_e/k_s_ - theta_o_;

    actual_twist = grasp_point_velocity_kdl.GetTwist();
    tf::twistKDLToEigen(actual_twist, actual_twist_eigen);

    actual_commands << actual_twist_eigen.block<3,1>(0,0).dot(surface_tangent), actual_twist_eigen.block<3,1>(0,0).dot(surface_normal), actual_twist_eigen.block<3,1>(3,0).dot(rotation_axis);
    Eigen::Vector3d variances = ekf_estimator_.getVariance();
    x_hat_ = ekf_estimator_.estimate(actual_commands, y, x_e_, dt.toSec());

    e = x_d_ - x_hat_;

    // Compute the ground truth from the known length and known surface
    double real_x1, real_x2, real_theta1, real_theta2;
    double A, B, C, line_displacement, center_x, center_y;
    center_x = x_e_[0];
    center_y = x_e_[1];

    line_displacement = 0;
    A = 1;
    B = -2*center_x;
    C = center_y*center_y - hardcoded_length_*hardcoded_length_ + center_x*center_x - 2*line_displacement*center_y + line_displacement*line_displacement;

    real_x1 = (-B + std::sqrt(B*B - 4*A*C))/(2*A);
    real_theta1 = std::atan(-center_y/(real_x1 - center_x));
    real_x2 = (-B - std::sqrt(B*B - 4*A*C))/(2*A);
    real_theta2 = std::atan(-center_y/(real_x2 - center_x));

    feedback_.x_c_1 = real_x1;
    feedback_.theta_c_1 = real_theta1;
    feedback_.x_c_2 = real_x2;
    feedback_.theta_c_2 = real_theta2;
    feedback_.x_c_hat = x_hat_[0];
    feedback_.x_d = x_d_[0];
    feedback_.var_x = variances[0];
    feedback_.theta_c_hat = x_hat_[1];
    feedback_.theta_d = x_d_[1];
    feedback_.var_theta = variances[1];
    feedback_.f_c_hat = x_hat_[2];
    feedback_.f_c = force_e;
    feedback_.f_d = x_d_[2];
    feedback_.var_f = variances[2];
    feedback_.torque_c = torque_e;
    feedback_.f_e.x = measured_wrench_[0];
    feedback_.f_e.y = measured_wrench_[1];
    feedback_.f_e.z = measured_wrench_[2];
    feedback_.tau_e.x = measured_wrench_[3];
    feedback_.tau_e.y = measured_wrench_[4];
    feedback_.tau_e.z = measured_wrench_[5];
    feedback_.error_x = e[0];
    feedback_.error_theta = e[1];
    feedback_.error_force = e[2];
    feedback_.commanded_x = commands[0];
    feedback_.commanded_y = commands[1];
    feedback_.commanded_rot = commands[2];
    feedback_.rot_gains.x = rot_gains_.vector.x;
    feedback_.rot_gains.y = rot_gains_.vector.y;
    feedback_.rot_gains.z = rot_gains_.vector.z;

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

    // input_twist(3) += rot_gains_.vector.x*twist_error(3); // TODO: I have an issue with rotating around the world frame x
    input_twist(4) += rot_gains_.vector.y*twist_error(4);
    input_twist(5) += rot_gains_.vector.z*twist_error(5);

    tf::twistKDLToMsg(input_twist, feedback_.corrected_twist);

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

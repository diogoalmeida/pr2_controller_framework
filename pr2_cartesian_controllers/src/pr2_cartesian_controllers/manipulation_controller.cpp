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
  }

  void ManipulationController::preemptCB()
  {
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    ROS_WARN("Manipulation controller preempted!");
  }

  void ManipulationController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::ManipulationControllerGoal> goal = action_server_->acceptNewGoal();
    geometry_msgs::PoseStamped pose_in, pose_out;
    Eigen::Vector3d init_x;
    double x_d, theta_d, f_d;

    finished_acquiring_goal_ = false;
    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    if(goal->arm < 0 || goal->arm >= NUM_ARMS)
    {
      ROS_ERROR("Received a goal where the requested arm (%d) does not exist", goal->arm);
      action_server_->setAborted();
      return;
    }

    arm_index_ = goal->arm;
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

    initTwistController(comp_gains_, base_link_, pose_in.header.frame_id);
    listener_.waitForTransform(pose_in.header.frame_id, base_link_, ros::Time(0), ros::Duration(wait_for_tf_time_));

    try
    {
      pose_in.header.stamp = ros::Time(0);
      listener_.transformPose(base_link_, pose_in, pose_out);
      tf::poseMsgToEigen(pose_out.pose, surface_frame_);

      tf::vectorMsgToEigen(goal->debug_eef_to_grasp, debug_eef_to_grasp_eig_);
      debug_x_ = goal->debug_twist.linear.x;
      debug_y_ = goal->debug_twist.linear.y;
      debug_rot_ = goal->debug_twist.angular.z;

      // get the relationship between kinematic chain end-effector and
      // tool-tip (grasping point)
      pose_in.header.frame_id = ft_frame_id_[arm_index_];
      pose_in.header.stamp = ros::Time(0); // get latest available

      pose_in.pose.position.x = 0;
      pose_in.pose.position.y = 0;
      pose_in.pose.position.z = 0;

      pose_in.pose.orientation.x = 0;
      pose_in.pose.orientation.y = 0;
      pose_in.pose.orientation.z = 0;
      pose_in.pose.orientation.w = 1;
      listener_.transformPose(end_effector_link_[arm_index_], pose_in, pose_out);
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
          Eigen::Vector3d y_pos = surface_frame_.translation() + surface_frame_vertical_offset_*surface_frame_.rotation().block<3,1>(0,2);
          x_d_eigen = y_pos + (x_d_[0] + surface_frame_horizontal_offset_)*surface_frame_.rotation().block<3,1>(0,0);
          x_c_eigen = y_pos + (x_hat_[0] + surface_frame_horizontal_offset_)*surface_frame_.rotation().block<3,1>(0,0);
          x_real_eigen = y_pos + (feedback_.x_c_2 + surface_frame_horizontal_offset_)*surface_frame_.rotation().block<3,1>(0,0);
          r_d = cos(x_d_[1])*surface_frame_.rotation().block<3,1>(0,0) + sin(x_d_[1])*surface_frame_.rotation().block<3,1>(0,2); // r = cos(theta)*x + sin(theta)*z
          r_1 = cos(x_hat_[1])*surface_frame_.rotation().block<3,1>(0,0) + sin(x_hat_[1])*surface_frame_.rotation().block<3,1>(0,2);

          double theta = std::atan2(x_e_[1] - surface_frame_vertical_offset_, x_e_[0] - surface_frame_horizontal_offset_ - feedback_.x_c_2);
          feedback_.theta_c_2 = theta;
          r_real = cos(theta)*surface_frame_.rotation().block<3,1>(0,0) + sin(theta)*surface_frame_.rotation().block<3,1>(0,2);
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

  bool ManipulationController::loadParams()
  {
    if (!getParam("/manipulation_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/estimate_length", estimate_length_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/hardcoded_length", hardcoded_length_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/compensation_gains", comp_gains_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/wait_for_tf_time", wait_for_tf_time_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/spring_constant", k_s_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/estimator/initial_x_offset", init_x_offset_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/estimator/initial_theta_offset", init_theta_offset_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/x_offset", x_o_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/surface_frame_vertical_offset", surface_frame_vertical_offset_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/surface_frame_horizontal_offset", surface_frame_horizontal_offset_))
    {
      return false;
    }

    if (!getParam("/manipulation_controller/initial_angle_offset", theta_o_))
    {
      return false;
    }

    if (comp_gains_.size() != 6)
    {
      ROS_ERROR("The rotational gains vector must have length 6! (Has length %zu)", comp_gains_.size());
      return false;
    }

    x_hat_ = Eigen::VectorXd(4);
    x_hat_ << 0, 0, 0, k_s_;

    return true;
  }

  sensor_msgs::JointState ManipulationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output;
    KDL::Frame end_effector_kdl, grasp_point_kdl, surface_frame_to_grasp, surface_frame_kdl;
    KDL::FrameVel end_effector_velocity_kdl, grasp_point_velocity_kdl;
    KDL::Twist input_twist, twist_error, actual_twist;
    Eigen::Affine3d surface_frame_to_grasp_eig;
    Eigen::Vector3d rotation_axis, surface_tangent, surface_normal, force, torque, commands, origin, eef_to_grasp_eig, velocity_command, velocity_eef, actual_commands;
    Eigen::Vector3d u, e, temp, surface_tangent_in_grasp, surface_normal_in_grasp;
    Eigen::VectorXd y(3);
    double torque_e, f_e_y, f_e_x;
    Eigen::Matrix3d inv_g, skew;
    Eigen::Matrix<double, 6, 1> twist_eig, actual_twist_eigen;

    if (!action_server_->isActive() || !finished_acquiring_goal_) // TODO: should be moved to parent class
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    KDL::JntArray commanded_joint_velocities(chain_[arm_index_].getNrOfJoints());
    if(!getChainJointState(current_state, chain_[arm_index_], joint_positions_[arm_index_], joint_velocities_[arm_index_]))
    {
      ROS_ERROR("Failed to get the chain joint state. Aborting.");
      action_server_->setAborted();
      lastState(current_state);
    }

    fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], end_effector_kdl);
    fkvel_[arm_index_]->JntToCart(joint_velocities_[arm_index_], end_effector_velocity_kdl);
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
      fkpos_[arm_index_]->JntToCart(joint_positions_[arm_index_], initial_pose_); // base_link
      x_hat_[0] = x_e_[0] + init_x_offset_; // initial x_c estimate, made different from x_e_ to avoid dx = 0
      x_hat_[1] = x_e_[2] + init_theta_offset_;
      x_hat_[2] = measured_wrench_[arm_index_].block<3,1>(0,0).dot(surface_normal_in_grasp);
      x_hat_[3] = k_s_;
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

    torque_e = measured_wrench_[arm_index_].block<3,1>(3,0).dot(computeSkewSymmetric(surface_normal_in_grasp)*surface_tangent_in_grasp);
    // torque_e = -measured_wrench_.block<3,1>(3,0).norm();
    f_e_y = measured_wrench_[arm_index_].block<3,1>(0,0).dot(surface_normal_in_grasp);
    f_e_x = measured_wrench_[arm_index_].block<3,1>(0,0).dot(surface_tangent_in_grasp);


    // compute the measurements vector
    y << torque_e/f_e_y,
         x_e_[2] - theta_o_,
         f_e_y;

    actual_twist = grasp_point_velocity_kdl.GetTwist();
    tf::twistKDLToEigen(actual_twist, actual_twist_eigen);

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
    real_theta1 = std::atan2(center_y, (center_x - real_x2));
    real_x2 = (-B - std::sqrt(B*B - 4*A*C))/(2*A);
    real_theta2 = std::atan2(center_y, (center_x - real_x2));

    if (debug_twist_)
    {
      commands << debug_x_, debug_y_, debug_rot_;
    }
    else
    {
      x_hat_ << real_x2, real_theta2, x_hat_[2], x_hat_[3];
      commands = controller_.compute(x_d_, x_hat_, x_e_);
    }

    actual_commands << actual_twist_eigen.block<3,1>(0,0).dot(surface_tangent), actual_twist_eigen.block<3,1>(0,0).dot(surface_normal), actual_twist_eigen.block<3,1>(3,0).dot(rotation_axis);
    x_hat_ = ekf_estimator_.estimate(actual_commands, y, x_e_, dt.toSec());
    x_hat_[0] += x_o_;

    k_s_ = x_hat_[3];
    Eigen::Vector3d x_red;
    x_red << x_hat_[0], x_hat_[1], x_hat_[2];
    e = x_d_ - x_red;

    feedback_.x_c_1 = real_x1;
    feedback_.theta_c_1 = real_theta1;
    feedback_.x_c_2 = real_x2;
    feedback_.theta_c_2 = real_theta2;
    feedback_.x_c_hat = x_hat_[0];
    feedback_.x_d = x_d_[0];
    feedback_.theta_c_hat = x_hat_[1];
    feedback_.theta_d = x_d_[1];
    feedback_.f_c_y_hat = x_hat_[2];
    feedback_.f_c_y = f_e_y;
    feedback_.f_c_x_hat = 0;
    feedback_.f_c_x = f_e_x;
    feedback_.f_d = x_d_[2];
    feedback_.k_s = k_s_;
    feedback_.torque_c = torque_e;
    feedback_.f_e.x = measured_wrench_[arm_index_][0];
    feedback_.f_e.y = measured_wrench_[arm_index_][1];
    feedback_.f_e.z = measured_wrench_[arm_index_][2];
    feedback_.tau_e.x = measured_wrench_[arm_index_][3];
    feedback_.tau_e.y = measured_wrench_[arm_index_][4];
    feedback_.tau_e.z = measured_wrench_[arm_index_][5];
    feedback_.error_x = e[0];
    feedback_.error_theta = e[1];
    feedback_.error_force = e[2];
    feedback_.commanded_x = commands[0];
    feedback_.commanded_y = commands[1];
    feedback_.commanded_rot = commands[2];
    feedback_.x_e = x_e_[0];
    feedback_.y_e = x_e_[1];
    feedback_.theta_e = x_e_[2];
    feedback_.x_e_dot = actual_commands[0];
    feedback_.y_e_dot = actual_commands[1];
    feedback_.theta_e_dot = actual_commands[2];

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

    input_twist += twist_controller_->computeError(end_effector_kdl, initial_pose_);

    tf::twistKDLToMsg(input_twist, feedback_.corrected_twist);

    ikvel_[arm_index_]->CartToJnt(joint_positions_[arm_index_], input_twist, commanded_joint_velocities);
    control_output = current_state;

    int joint_index;
    for (unsigned long i = 0; i < current_state.name.size(); i++)
    {
      if (hasJoint(chain_[arm_index_], current_state.name[i]))
      {
        joint_index = getJointIndex(actuated_joint_names_[arm_index_], current_state.name[i]);
        control_output.position[i] = joint_positions_[arm_index_](joint_index) + commanded_joint_velocities(joint_index)*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities(joint_index);
      }
    }

    return control_output;
  }

  Eigen::Matrix3d ManipulationController::computeSkewSymmetric(const Eigen::Vector3d &v)
  {
    Eigen::Matrix3d S;

    S << 0,    -v(2),  v(1),
         v(2),  0   , -v(0),
        -v(1),  v(0),  0;

    return S;
  }
}

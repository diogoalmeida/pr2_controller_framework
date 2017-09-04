#include <pr2_cartesian_controllers/mechanism_identification_controller.hpp>

namespace cartesian_controllers {

  MechanismIdentificationController::MechanismIdentificationController() : ControllerTemplate<pr2_cartesian_controllers::MechanismIdentificationAction,
                                                pr2_cartesian_controllers::MechanismIdentificationFeedback,
                                                pr2_cartesian_controllers::MechanismIdentificationResult>(), eef_to_grasp_(2), sensor_frame_to_base_(2)
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one identification action run
    startActionlib();
    finished_acquiring_goal_ = false;
    pc_pub_ = nh_.advertise<visualization_msgs::Marker>("pc", 1);
    pc_est_pub_ = nh_.advertise<visualization_msgs::Marker>("pc_est", 1);
    p1_pub_ = nh_.advertise<visualization_msgs::Marker>("p1", 1);
    p2_pub_ = nh_.advertise<visualization_msgs::Marker>("p2", 1);
    r1_pub_ = nh_.advertise<visualization_msgs::Marker>("r1", 1);
    r1_est_pub_ = nh_.advertise<visualization_msgs::Marker>("r1_est", 1);
    r2_pub_ = nh_.advertise<visualization_msgs::Marker>("r2", 1);
    r2_est_pub_ = nh_.advertise<visualization_msgs::Marker>("r2_est", 1);
    trans_pub_ = nh_.advertise<visualization_msgs::Marker>("trans", 1);
    trans_est_pub_ = nh_.advertise<visualization_msgs::Marker>("trans_est", 1);
    rot_pub_ = nh_.advertise<visualization_msgs::Marker>("rot", 1);
    rot_est_pub_ = nh_.advertise<visualization_msgs::Marker>("rot_est", 1);
    init_t_error_ = 0;
    init_k_error_ = 0;
    rotation_t_ = 0;
    rotation_k_ = 0;
    rotation_n_ = 0;
    use_nullspace_ = false;
    has_joint_positions_ = false;
    use_kalman_gain_ = true;
    wrench2_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("surface_frame_wrench", 1);
    relative_twist_publisher_ = nh_.advertise<geometry_msgs::WrenchStamped>("commanded_relative_twist", 1);
    feedback_thread_ = boost::thread(boost::bind(&MechanismIdentificationController::publishFeedback, this));
    cfg_callback_ = boost::bind(&MechanismIdentificationController::dynamicReconfigureCallback, this, _1, _2);
  }

  MechanismIdentificationController::~MechanismIdentificationController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
  }

  void MechanismIdentificationController::preemptCB()
  {
    ROS_WARN("Mechanism identification preempt callback");
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    has_joint_positions_ = false;
    ROS_WARN("Mechanism identification controller preempted!");
  }

  void MechanismIdentificationController::dynamicReconfigureCallback(const pr2_cartesian_controllers::MechanismIdentificationConfig &config, uint32_t level)
  {
    goal_force_ = config.force;
    vd_amp_ = config.vd_amp;
    vd_freq_ = config.vd_freq;
    wd_amp_ = config.wd_amp;
    wd_freq_ = config.wd_freq;
    use_nullspace_ = config.use_nullspace;
    rotation_t_ = config.rotation_t;
    rotation_k_ = config.rotation_k;
    rotation_n_ = config.rotation_n;
    kalman_estimator_.setObserverGain(config.constant_observer_gain);
    if (ects_controller_)
    {
      ects_controller_->setNullspaceGain(config.nullspace_gain);
    }
    use_kalman_gain_ = config.use_kalman_gain;
  }

  void MechanismIdentificationController::goalCB()
  {
    ROS_INFO("Mechanism goal callback");
    boost::shared_ptr<const pr2_cartesian_controllers::MechanismIdentificationGoal> goal = action_server_->acceptNewGoal();
    {
      boost::lock_guard<boost::mutex>guard(reference_mutex_);
      finished_acquiring_goal_ = false;
      has_joint_positions_ = false;
    }
    
    ROS_INFO("Got mutex");

    cfg_server_.reset(new dynamic_reconfigure::Server<pr2_cartesian_controllers::MechanismIdentificationConfig>(ros::NodeHandle(ros::this_node::getName() + "/mechanism_identification_config")));
    cfg_server_->setCallback(cfg_callback_);
    
    ROS_INFO("ECTS");
    
    ects_controller_.reset(new manipulation_algorithms::ECTSController(chain_[0], chain_[1]));

    ROS_INFO("Initialized cfg; ects");

    if(!ects_controller_->getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    if(!adaptive_controller_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    if(!kalman_estimator_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    if (!rot_estimator_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }
    
    ROS_INFO("Got parameters");

    rod_arm_ = goal->rod_arm;
    surface_arm_ = goal->surface_arm;
    goal_force_ = goal->goal_force;
    vd_amp_ = goal->vd_amplitude;
    vd_freq_ = goal->vd_frequency;
    wd_amp_ = goal->wd_amplitude;
    wd_freq_ = goal->wd_frequency;
    use_nullspace_ = goal->use_nullspace;
    use_estimates_ = goal->use_estimates;
    init_t_error_ = goal->init_t_error;
    init_k_error_ = goal->init_k_error;
    init_pc_error_ = goal->init_pc_error;
    
    angle_gen_ = std::uniform_real_distribution<double>(0, 2*M_PI);
    
    ROS_INFO("Parsed goal");
    
    ects_controller_->setNullspaceGain(goal->nullspace_gain);
    ects_controller_->setAlpha(goal->alpha);
    initTwistController(comp_gains_, base_link_, ft_frame_id_[surface_arm_]);
    geometry_msgs::PoseStamped pose_in, pose_out;
    
    ROS_INFO("Finished initialization");
    
    try
    {
      // get the relationship between kinematic chain end-effector and
      // tool-tip (grasping point)
      for (int i = 0; i < NUM_ARMS; i++)
      {
        pose_in.header.frame_id = ft_frame_id_[i];
        pose_in.header.stamp = ros::Time(0); // get latest available

        pose_in.pose.position.x = 0;
        pose_in.pose.position.y = 0;
        pose_in.pose.position.z = 0;

        pose_in.pose.orientation.x = 0;
        pose_in.pose.orientation.y = 0;
        pose_in.pose.orientation.z = 0;
        pose_in.pose.orientation.w = 1;
        listener_.transformPose(end_effector_link_[i], pose_in, pose_out);
        tf::poseMsgToKDL(pose_out.pose, eef_to_grasp_[i]);

        listener_.transformPose(base_link_, pose_in, pose_out);
        tf::poseMsgToKDL(pose_out.pose, sensor_frame_to_base_[i]);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
      action_server_->setAborted();
      return;
    }
    
    ROS_INFO("Got tf");
    
    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      finished_acquiring_goal_ = true;
    }

    ROS_INFO("Mechanism identification controller server received a goal!");
  }

  void MechanismIdentificationController::publishFeedback()
  {
    visualization_msgs::Marker pc_marker, pc_est_marker, p1_marker, p2_marker, r1_marker, r1_est_marker, r2_marker, r2_est_marker, trans_marker, rot_marker, trans_est_marker, rot_est_marker;
    geometry_msgs::Vector3 r_vec, r1, r2, p1, p2, pc, pc_est, t, t_est, k, k_est;
    Eigen::Vector3d linear_vel_eig, angular_vel_eig, force_e, torque_e, force_d, torque_d, normal;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    geometry_msgs::WrenchStamped surface_wrench, force_control_twist;
    tf::Transform pc_transform;

    pc_marker.header.frame_id = chain_base_link_;
    pc_marker.ns = std::string("mechanism_identification");
    pc_marker.type = pc_marker.SPHERE;
    pc_marker.action = pc_marker.ADD;
    pc_marker.scale.x = 0.01;
    pc_marker.scale.y = 0.01;
    pc_marker.scale.z = 0.01;
    pc_marker.lifetime = ros::Duration(0);
    pc_marker.frame_locked = false;
    pc_marker.color.r = 1.0;
    pc_marker.color.a = 1.0;
    p1_marker = pc_marker;
    p1_marker.id = 1;
    p2_marker = pc_marker;
    p2_marker.id = 2;
    p1_marker.color.r = 0.0;
    p2_marker.color.r = 0.0;
    p1_marker.color.g = 1.0;
    p2_marker.color.g = 1.0;
    r1_marker = p1_marker;
    r1_marker.id = 3;
    r1_marker.scale.y = 0.005;
    r1_marker.scale.z = 0.005;
    r1_marker.type = r1_marker.ARROW;
    r2_marker = r1_marker;
    r2_marker.id = 4;
    r2_marker.color = pc_marker.color;
    trans_marker = r2_marker;
    trans_marker.id = 5;
    trans_marker.color.r = 1.0;
    trans_marker.color.g = 0.0;
    trans_marker.scale.x = 0.01;
    trans_marker.scale.y = 0.01;
    trans_marker.scale.z = 0.01;
    rot_marker = trans_marker;
    rot_marker.color.r = 0.0;
    rot_marker.color.b = 1.0;
    pc_est_marker = pc_marker;
    pc_est_marker.id = 6;
    pc_est_marker.color.g = 1.0;
    r1_est_marker = r1_marker;
    r1_est_marker.id = 7;
    r1_est_marker.color.r = 0.5;
    r2_est_marker = r2_marker;
    r2_est_marker.id = 8;
    r2_est_marker.color.r = 0.0;
    r2_est_marker.color.b = 0.5;
    trans_est_marker = trans_marker;
    trans_est_marker.color.r = 0.5;
    trans_est_marker.color.b = 0.5;
    trans_est_marker.color.g = 0.0;
    trans_est_marker.id = 9;
    rot_est_marker = trans_est_marker;
    rot_est_marker.id = 10;
    rot_est_marker.color.r = 0.0;
    rot_est_marker.color.g = 0.5;

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive() && finished_acquiring_goal_)
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          pc_marker.header.stamp = ros::Time::now();
          p1_marker.header.stamp = ros::Time::now();
          p2_marker.header.stamp = ros::Time::now();
          r1_marker.header.stamp = ros::Time::now();
          r2_marker.header.stamp = ros::Time::now();
          trans_marker.header.stamp = ros::Time::now();
          rot_marker.header.stamp = ros::Time::now();

          tf::poseEigenToMsg(pc_, pc_marker.pose);
          tf::poseEigenToMsg(pc_est_, pc_est_marker.pose);
          tf::poseEigenToMsg(p1_, p1_marker.pose);
          tf::poseEigenToMsg(p2_, p2_marker.pose);
          getMarkerPoints(p1_.translation(), pc_.translation(), r1_marker);
          getMarkerPoints(p1_.translation(), pc_est_.translation(), r1_est_marker);
          getMarkerPoints(p2_.translation(), pc_.translation(), r2_marker);
          getMarkerPoints(p2_.translation(), pc_est_.translation(), r2_est_marker);
          getMarkerPoints(pc_.translation(), pc_.translation() + 0.1*translational_dof_ground_, trans_marker);
          getMarkerPoints(pc_.translation(), pc_.translation() + 0.1*rotational_dof_ground_, rot_marker);
          getMarkerPoints(pc_.translation(), pc_.translation() + 0.1*translational_dof_est_, trans_est_marker);
          getMarkerPoints(pc_.translation(), pc_.translation() + 0.1*rotational_dof_est_, rot_est_marker);

          pc_pub_.publish(pc_marker);
          pc_est_pub_.publish(pc_est_marker);
          p1_pub_.publish(p1_marker);
          p2_pub_.publish(p2_marker);
          r1_pub_.publish(r1_marker);
          r1_est_pub_.publish(r1_est_marker);
          r2_pub_.publish(r2_marker);
          r2_est_pub_.publish(r2_est_marker);
          trans_pub_.publish(trans_marker);
          rot_pub_.publish(rot_marker);
          trans_est_pub_.publish(trans_est_marker);
          rot_est_pub_.publish(rot_est_marker);

          pc_transform.setOrigin( tf::Vector3(pc_.translation()[0], pc_.translation()[1], pc_.translation()[2]));
          tf::Quaternion pc_orientation(0, 0, 0);
          pc_transform.setRotation(pc_orientation);
          broadcaster_.sendTransform(tf::StampedTransform(pc_transform, ros::Time::now(), chain_base_link_, "mechanism_pc"));

          adaptive_controller_.getForceControlValues(linear_vel_eig, angular_vel_eig);
          force_control_twist.header.frame_id = "mechanism_pc";
          force_control_twist.header.stamp = ros::Time::now();
          KDL::Twist twist_adaptive;
          Eigen::Matrix<double, 6, 1> twist_adaptive_eig;
          twist_adaptive_eig.block<3,1>(0,0) = linear_vel_eig;
          twist_adaptive_eig.block<3,1>(3,0) = angular_vel_eig;
          tf::twistEigenToKDL(twist_adaptive_eig, twist_adaptive);
          twist_adaptive = sensor_frame_to_base_[surface_arm_].M*twist_adaptive;
          tf::twistKDLToEigen(twist_adaptive, twist_adaptive_eig);
          tf::vectorEigenToMsg(twist_adaptive_eig.block<3,1>(0,0), force_control_twist.wrench.force);
          tf::vectorEigenToMsg(twist_adaptive_eig.block<3,1>(3,0), force_control_twist.wrench.torque);
          relative_twist_publisher_.publish(force_control_twist);

          tf::vectorEigenToMsg(pc_.translation() - p1_.translation(), feedback_.r1);
          tf::vectorEigenToMsg(pc_.translation() - p2_.translation(), feedback_.r2);
          tf::vectorEigenToMsg(p1_.translation(), feedback_.p1);
          tf::vectorEigenToMsg(pc_.translation(), feedback_.pc);
          tf::vectorEigenToMsg(p2_.translation(), feedback_.p2);
          surface_wrench.header.frame_id = ft_frame_id_[surface_arm_];
          tf::wrenchEigenToMsg(wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]), surface_wrench.wrench);
          surface_wrench.header.stamp = ros::Time::now();
          feedback_.surface_wrench = surface_wrench;
          wrench2_pub_.publish(surface_wrench);

          feedback_.task_compatibility = ects_controller_->getTaskCompatibility();
          feedback_.alpha = ects_controller_->getAlpha();
          feedback_.absolute_twist.header.stamp = ros::Time::now();
          feedback_.relative_twist.header.stamp = ros::Time::now();
          adaptive_controller_.getErrors(force_e, torque_e, force_d, torque_d);
          
          tf::vectorEigenToMsg(p1_.translation(), feedback_.p1);
          tf::vectorEigenToMsg(p2_.translation(), feedback_.p2);
          tf::vectorEigenToMsg(pc_.translation(), feedback_.pc);
          tf::vectorEigenToMsg(pc_est_.translation(), feedback_.pc_est);
          tf::vectorEigenToMsg(translational_dof_ground_, feedback_.t);
          tf::vectorEigenToMsg(translational_dof_est_, feedback_.t_est);
          tf::vectorEigenToMsg(rotational_dof_ground_, feedback_.k);
          tf::vectorEigenToMsg(rotational_dof_est_, feedback_.k_est);
          feedback_.force_error = force_e.norm();
          feedback_.force_d = force_d.norm();
          feedback_.torque_error = torque_e.norm();
          feedback_.torque_d = torque_d.norm();
          feedback_.translational_angle_error = std::acos(translational_dof_ground_.dot(translational_dof_est_));
          feedback_.rotational_angle_error = std::acos(rotational_dof_ground_.dot(rotational_dof_est_));
          
          // normal = translational_dof_ground_.cross(rotational_dof_ground_);
          feedback_.pc_distance_error = ((I - rotational_dof_ground_*rotational_dof_ground_.transpose())*(pc_est_.translation() - pc_.translation())).norm();
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

  bool MechanismIdentificationController::loadParams()
  {
    if (!getParam("/mechanism_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/mechanism_controller/rotational_gains", comp_gains_))
    {
      return false;
    }

    if (!getParam("/mechanism_controller/use_estimates", use_estimates_))
    {
      return false;
    }

    if (!getParam("/mechanism_controller/rod_length", rod_length_))
    {
      return false;
    }

    if (!getParam("/mechanism_controller/joint_error_lim", joint_error_lim_))
    {
      return false;
    }

    if(!getParam("/mechanism_controller/estimator/adjust_x_force", adjust_x_force_))
    {
      return false;
    }

    if(!getParam("/mechanism_controller/estimator/adjust_y_force", adjust_y_force_))
    {
      return false;
    }

    if(!getParam("/mechanism_controller/estimator/adjust_z_force", adjust_z_force_))
    {
      return false;
    }

    return true;
  }

  sensor_msgs::JointState MechanismIdentificationController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output = current_state;
    std::vector<KDL::Frame> eef_grasp_kdl(2), eef_kdl(2);
    std::vector<KDL::FrameVel> eef_vel_kdl(2), eef_grasp_vel_kdl(2);
    std::vector<KDL::Twist> eef_twist(2);
    std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2);
    std::vector<Eigen::Matrix<double, 6, 1> > eef_twist_eig(2);
    std::vector<Eigen::Matrix<double, 7, 1> > q_dot(2);
    std::vector<KDL::Twist> command_twist(2);
    std::vector<KDL::JntArray> commanded_joint_velocities(2);
    std::vector<Eigen::Matrix<double, 6, 7> > jacobian(2);
    Eigen::Vector3d p1, p2, eef1, eef2,
                    contact_force, contact_torque, surface_tangent_in_grasp, rotation_axis,
                    out_vel_lin, out_vel_ang, surface_normal; // all in the base_link frame
    Eigen::Matrix<double, 12, 1> ects_twist = Eigen::Matrix<double, 12, 1>::Zero(), transmission_direction;
    Eigen::Matrix<double, 14, 1> joint_commands;
    KDL::Twist comp_twist;
    Eigen::Matrix<double, 6, 1> comp_twist_eig;
    KDL::Jacobian kdl_jac(7);
    Eigen::Vector3d rot_ground_in_frame = Eigen::Vector3d::Zero(), trans_ground_in_frame = Eigen::Vector3d::Zero();

    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    if (!action_server_->isActive() || !finished_acquiring_goal_) // TODO: should be moved to parent class
    {
      return lastState(current_state);
    }

    // TODO: This should be handled in the template class
    has_state_ = false;

    if(!getChainJointState(current_state, chain_[surface_arm_], joint_positions_[surface_arm_], joint_velocities_[surface_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the surface arm. Aborting.");
      action_server_->setAborted();
      return lastState(current_state);
    }

    if(!getChainJointState(current_state, chain_[rod_arm_], joint_positions_[rod_arm_], joint_velocities_[rod_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the rod arm. Aborting.");
      action_server_->setAborted();
      return lastState(current_state);
    }

    if (!has_joint_positions_)
    {
      target_joint_positions_ = joint_positions_;
      has_joint_positions_ = true;
    }

    control_output = lastState(current_state, rod_arm_);

    for (int arm = 0; arm < 2; arm++) // Compute forward kinematics and convert to grasp frame
    {
      fkpos_[arm]->JntToCart(joint_positions_[arm], eef_kdl[arm]);
      fkvel_[arm]->JntToCart(joint_velocities_[arm], eef_vel_kdl[arm]);
      q_dot[arm] = joint_velocities_[arm].qdot.data;
      eef_grasp_kdl[arm] = eef_kdl[arm]*eef_to_grasp_[arm];
      eef_grasp_vel_kdl[arm] = eef_vel_kdl[arm]*eef_to_grasp_[arm];
      eef_twist[arm] = eef_grasp_kdl[arm].Inverse()*eef_grasp_vel_kdl[arm].GetTwist();
      tf::transformKDLToEigen(eef_to_grasp_[arm], eef_to_grasp_eig[arm]);
      tf::transformKDLToEigen(eef_grasp_kdl[arm], grasp_point_frame[arm]);
      tf::twistKDLToEigen(eef_grasp_vel_kdl[arm].GetTwist(), eef_twist_eig[arm]);
      commanded_joint_velocities[arm] = KDL::JntArray(chain_[arm].getNrOfJoints());
      jac_solver_[arm]->JntToJac(joint_positions_[arm], kdl_jac);
      jacobian[arm] = kdl_jac.data;
    }

    rotational_dof_ground_  = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,1);
    rot_ground_in_frame[1] = 1;
    translational_dof_ground_ = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,0);
    surface_normal = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,2);
    trans_ground_in_frame[0] = 1;
    p1 = grasp_point_frame[rod_arm_].translation();
    p2 = grasp_point_frame[surface_arm_].translation();

    for(int i = 0; i < 3; i++)
    {
      eef1[i] = eef_kdl[rod_arm_].p(i);
      eef2[i] = eef_kdl[surface_arm_].p(i);
    }

    contact_force = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(0,0);
    contact_torque = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(3,0);
    p1_ = grasp_point_frame[rod_arm_];
    p2_ = grasp_point_frame[surface_arm_];
    pc_.translation() = p1_.translation() + rod_length_*p1_.matrix().block<3,1>(0,0); // it is assumed (just in the ground truth data) that the rod is aligned with the x axis of the grasp frame
    
    if (!has_initial_)
    {  
      std::mt19937 gen(rd_());
      double t_angle = angle_gen_(gen);
      double k_angle = angle_gen_(gen);
      double theta_sphere = angle_gen_(gen);
      double phi_sphere = angle_gen_(gen);
      Eigen::Vector3d t_cone_vector, k_cone_vector, sphere_vector;
      Eigen::Quaterniond base_rot;
      
      // t_cone_vector << cos(init_t_error_)*cos(t_angle), cos(init_t_error_)*sin(t_angle), -sin(init_t_error_);
      // t_cone_vector << cos(init_t_error_), sin(init_t_error_)*sin(t_angle), std::abs(cos(t_angle)*sin(init_t_error_));
      t_cone_vector << cos(init_t_error_), 0, sin(init_t_error_);
      // k_cone_vector << cos(init_k_error_)*cos(k_angle), cos(init_k_error_)*sin(k_angle), -sin(init_k_error_);
      // k_cone_vector << sin(k_angle)*sin(init_k_error_), cos(init_k_error_), cos(k_angle)*sin(init_k_error_);
      k_cone_vector << 0, cos(init_k_error_), sin(init_k_error_);
      sphere_vector << cos(theta_sphere), sin(theta_sphere)*sin(phi_sphere), std::abs(cos(phi_sphere)*sin(theta_sphere));
      sphere_vector = base_rot*sphere_vector;
      pc_est_.translation() = p2_.translation() + init_pc_error_*sphere_vector;
      
      kalman_estimator_.initialize(pc_est_.translation());
      rot_estimator_.initialize(rotational_dof_est_);
      adaptive_controller_.initEstimates(t_cone_vector, k_cone_vector);
      tf::quaternionKDLToEigen (sensor_frame_to_base_[surface_arm_].M, base_rot);
      t_cone_vector = base_rot*t_cone_vector;
      k_cone_vector = base_rot*k_cone_vector;
      rotational_dof_est_ = k_cone_vector;
      translational_dof_est_ = t_cone_vector;
      adaptive_controller_.setReferenceForce(goal_force_);
      elapsed_ = ros::Time(0);
      has_initial_ = true;
    }

    elapsed_ += dt;
    pc_.linear() =  p1_.linear();
    pc_est_.linear() =  p1_.linear();

    KDL::Wrench wrench_kdl;
    Eigen::Matrix<double, 6, 1> wrench_eig;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    // Get wrench in surface frame written in base frame coordinates
    wrench_eig = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]);
    wrench_eig[0] = adjust_x_force_*wrench_eig[0]; // compensates for miscalibration
    wrench_eig[1] = adjust_y_force_*wrench_eig[1];
    wrench_eig[2] = adjust_z_force_*wrench_eig[2]; 
    wrench_eig[3] = wrench_eig[3]/adjust_z_force_;
    wrench_eig[4] = wrench_eig[4]/adjust_z_force_;
    wrench_eig[5] = wrench_eig[5]/adjust_y_force_;
    tf::wrenchEigenToKDL(wrench_eig, wrench_kdl);
    wrench_kdl = sensor_frame_to_base_[surface_arm_].M*wrench_kdl;
    tf::wrenchKDLToEigen(wrench_kdl, wrench_eig);
    wrench_eig.block<3,1>(0,0) = wrench_eig.block<3,1>(0,0).dot(surface_normal)*surface_normal;
    wrench_eig.block<3,1>(3,0) = wrench_eig.block<3,1>(0,0).dot(rotational_dof_ground_)*rotational_dof_ground_;
    // wrench_eig = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]);
    
    // surface_normal = translational_dof_ground_.cross(rotational_dof_ground_);
    // // Eigen::AngleAxisd rot_t_eig(rotation_t_, translational_dof_ground_);
    // // Eigen::AngleAxisd rot_k_eig(rotation_k_, rotational_dof_ground_);
    // // Eigen::AngleAxisd rot_n_eig(rotation_n_, surface_normal);
    // KDL::Rotation rot_t, rot_k, rot_n;
    // KDL::Vector t, k, n;
    //   
    // t.x(translational_dof_ground_[0]);
    // t.y(translational_dof_ground_[1]);
    // t.z(translational_dof_ground_[2]);
    // k.x(rotational_dof_ground_[0]);
    // k.y(rotational_dof_ground_[1]);
    // k.z(rotational_dof_ground_[2]);
    // n.x(surface_normal[0]);
    // n.y(surface_normal[1]);
    // n.z(surface_normal[2]);
    // 
    // rot_t = rot_t.Rot(t, rotation_t_);
    // rot_k = rot_k.Rot(k, rotation_k_);
    // rot_n = rot_n.Rot(n, rotation_n_);
    // 
    // wrench_kdl = rot_t*wrench_kdl;
    // wrench_kdl = rot_k*wrench_kdl;
    // wrench_kdl = rot_n*wrench_kdl;
    
    // tf::wrenchKDLToEigen(wrench_kdl, wrench_eig_modified_);
    
    wrench_eig_modified_ = wrench_eig;
    // wrench_eig_modified_.block<3, 1>(0,0) = wrench_eig.block<3, 1>(0,0).dot(surface_normal)*surface_normal;
    // wrench_eig_modified_.block<3, 1>(0,0) = (I - rotational_dof_ground_*rotational_dof_ground_.transpose() )*wrench_eig.block<3, 1>(0,0);
    // wrench_eig_modified_.block<3, 1>(3,0) = wrench_eig.block<3, 1>(3,0).dot(rotational_dof_ground_)*rotational_dof_ground_;

    if (use_estimates_)
    {
      // TODO
      if (use_kalman_gain_)
      {
        pc_est_.translation() = kalman_estimator_.estimate(p1_.translation(), eef_twist_eig[rod_arm_], p2_.translation(), wrench_eig, dt.toSec());
        // pc_est_.translation() = kalman_estimator_.estimate(p1_.translation(), eef_twist_eig[rod_arm_], p2_.translation(), wrench_eig_modified_, dt.toSec()); // HACK: Test KF without normal force  
        // pc_est_.translation() = kalman_estimator_.estimate(p1_.translation(), eef_twist_eig[rod_arm_], p2_.translation(), wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]), dt.toSec());
        // pc_est_.translation() = p2_.translation() + p2_.linear()*(pc_est_.translation() - p2_.translation());
      }
      else
      {
        pc_est_.translation() = kalman_estimator_.estimateConstant(p1_.translation(), eef_twist_eig[rod_arm_], p2_.translation(), wrench_eig, dt.toSec());
        // pc_est_.translation() = kalman_estimator_.estimateConstant(p1_.translation(), eef_twist_eig[rod_arm_], p2_.translation(), wrench_eig_modified_, dt.toSec());
      }

      // ects_twist.block<6,1>(6,0) = adaptive_controller_.control(wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]), vd_amp_*sin(2*M_PI*vd_freq_*elapsed_.toSec()), wd_amp_*sin(2*M_PI*wd_freq_*elapsed_.toSec()), dt.toSec());
      // wrench_eig.block<3, 1>(3,0) = wrench_eig.block<3, 1>(3,0).dot(rotational_dof_ground_)*rotational_dof_ground_;
      KDL::Twist twist_adaptive;
      KDL::Vector trans_est_kdl, rot_est_kdl, r_2_kdl;
      Eigen::Vector3d r_2 = pc_est_.translation() - p2_.translation(), w_r, discard;
      Eigen::Vector3d rot_est_eig_frame, trans_est_eig_frame;
      Eigen::Matrix<double, 6, 1> twist_adaptive_eig;
      double v_d, w_d;
      
      v_d = vd_amp_*sin(2*M_PI*vd_freq_*elapsed_.toSec());
      w_d = wd_amp_*sin(2*M_PI*wd_freq_*elapsed_.toSec());
      
      feedback_.v_d = v_d;
      feedback_.w_d = w_d;
      
      r_2_kdl.x(r_2[0]); 
      r_2_kdl.y(r_2[1]); 
      r_2_kdl.z(r_2[2]); 
      r_2_kdl = eef_grasp_kdl[surface_arm_].M.Inverse()*r_2_kdl;
      r_2[0] = r_2_kdl.x();
      r_2[1] = r_2_kdl.y();
      r_2[2] = r_2_kdl.z();
      
      twist_adaptive_eig = adaptive_controller_.control(wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]), r_2, v_d, w_d, dt.toSec());
      tf::twistEigenToKDL(twist_adaptive_eig, twist_adaptive);
      twist_adaptive = sensor_frame_to_base_[surface_arm_].M*twist_adaptive;
      tf::twistKDLToEigen(twist_adaptive, twist_adaptive_eig);
      ects_twist.block<6,1>(6,0) = twist_adaptive_eig;
      adaptive_controller_.getEstimates(trans_est_eig_frame, rot_est_eig_frame);
      trans_est_kdl.x(trans_est_eig_frame[0]);
      trans_est_kdl.y(trans_est_eig_frame[1]);
      trans_est_kdl.z(trans_est_eig_frame[2]);
      rot_est_kdl.x(rot_est_eig_frame[0]);
      rot_est_kdl.y(rot_est_eig_frame[1]);
      rot_est_kdl.z(rot_est_eig_frame[2]);
      trans_est_kdl = eef_grasp_kdl[surface_arm_].M*trans_est_kdl;
      rot_est_kdl = eef_grasp_kdl[surface_arm_].M*rot_est_kdl;
      translational_dof_est_[0] = trans_est_kdl.x();
      translational_dof_est_[1] = trans_est_kdl.y();
      translational_dof_est_[2] = trans_est_kdl.z();
      rotational_dof_est_[0] = rot_est_kdl.x();
      rotational_dof_est_[1] = rot_est_kdl.y();
      rotational_dof_est_[2] = rot_est_kdl.z();
      w_r = eef_twist_eig[surface_arm_].block<3,1>(3,0) - eef_twist_eig[rod_arm_].block<3,1>(3,0);
      
      // if (std::abs(w_d) > -0.005)
      // {
      //   rotational_dof_est_ = rot_estimator_.estimate(w_r, dt.toSec());
      // }
      // rot_est_kdl.x(rotational_dof_est_[0]);
      // rot_est_kdl.y(rotational_dof_est_[1]);
      // rot_est_kdl.z(rotational_dof_est_[2]);
      // rot_est_kdl = eef_grasp_kdl[surface_arm_].M.Inverse()*rot_est_kdl;
      // rot_est_eig_frame[0] = rot_est_kdl.x();
      // rot_est_eig_frame[1] = rot_est_kdl.y();
      // rot_est_eig_frame[2] = rot_est_kdl.z();
      // adaptive_controller_.initEstimates(trans_est_eig_frame, rot_est_eig_frame);
      
      // ects_twist.block<3,1>(6,0) = vd_amp_*sin(2*M_PI*vd_freq_*elapsed_.toSec())*translational_dof_ground_;
      // ects_twist.block<3,1>(9,0) = wd_amp_*sin(2*M_PI*wd_freq_*elapsed_.toSec())*rotational_dof_ground_;
    }
    else
    {
      // translational_dof_est_ = translational_dof_ground_;
      // rotational_dof_est_ = rotational_dof_ground_;
      // pc_est_ = pc_;
      ects_twist.block<3,1>(6,0) = vd_amp_*sin(2*M_PI*vd_freq_*elapsed_.toSec())*translational_dof_ground_;
      ects_twist.block<3,1>(9,0) = wd_amp_*sin(2*M_PI*wd_freq_*elapsed_.toSec())*rotational_dof_ground_;
    }

    ects_controller_->clearOptimizationDirections();
    if (use_nullspace_)
    {
      transmission_direction = Eigen::Matrix<double, 12, 1>::Zero();
      transmission_direction.block<3,1>(6,0) = translational_dof_est_;
      ects_controller_->addOptimizationDirection(transmission_direction);
      transmission_direction = Eigen::Matrix<double, 12, 1>::Zero();
      transmission_direction.block<3,1>(9,0) = rotational_dof_est_;
      ects_controller_->addOptimizationDirection(transmission_direction);
    }

    tf::twistEigenToMsg(ects_twist.block<6,1>(0,0), feedback_.absolute_twist.twist);
    tf::twistEigenToMsg(ects_twist.block<6,1>(6,0), feedback_.relative_twist.twist);
    feedback_.vs = ects_twist.block<3,1>(6,0).dot(translational_dof_est_);
    feedback_.vforce = ects_twist.block<3,1>(6,0).dot(surface_normal);
    feedback_.wr = ects_twist.block<3,1>(9,0).dot(rotational_dof_est_);
    
    comp_twist = twist_controller_->computeError(eef_grasp_kdl[rod_arm_], eef_grasp_kdl[surface_arm_]); // want to stay aligned with the surface_arm
    tf::twistKDLToEigen(comp_twist, comp_twist_eig);
    ects_twist.block<6,1>(6,0) += comp_twist_eig;
    // ikvel_[rod_arm_]->CartToJnt(joint_positions_[rod_arm_], comp_twist, commanded_joint_velocities[rod_arm_]);
    // comp_twist = twist_controller_->computeError(eef_grasp_kdl[surface_arm_], eef_grasp_kdl[rod_arm_]);
    // ikvel_[surface_arm_]->CartToJnt(joint_positions_[surface_arm_], comp_twist, commanded_joint_velocities[surface_arm_]);

    // tf::twistKDLToEigen(comp_twist.RefPoint(eef_to_grasp_[rod_arm_].p), comp_twist_eig);
    // joint_commands = ects_controller_->control(pc_.translation() - eef1, pc_.translation() - eef2, joint_positions_[rod_arm_], joint_positions_[surface_arm_], ects_twist.block<6,1>(0,0), ects_twist.block<6,1>(6,0));
    joint_commands = ects_controller_->control(pc_est_.translation() - eef1, pc_est_.translation() - eef2, joint_positions_[rod_arm_], joint_positions_[surface_arm_], ects_twist.block<6,1>(0,0), ects_twist.block<6,1>(6,0));

    for (unsigned int i = 0; i < 7; i++)
    {
      commanded_joint_velocities[rod_arm_](i) = joint_commands[i];
      commanded_joint_velocities[surface_arm_](i) = joint_commands[i + 7];
      
      if (std::abs(target_joint_positions_[rod_arm_](i) - joint_positions_[rod_arm_](i)) < joint_error_lim_)
      {
        target_joint_positions_[rod_arm_](i) += commanded_joint_velocities[rod_arm_](i)*dt.toSec();
      }

      if (std::abs(target_joint_positions_[surface_arm_](i) - joint_positions_[surface_arm_](i)) < joint_error_lim_)
      {
        target_joint_positions_[surface_arm_](i) += commanded_joint_velocities[surface_arm_](i)*dt.toSec();
      }

    }

    std::vector<int> joint_index(2, 0);
    for (unsigned long i = 0; i < current_state.name.size(); i++)
    {
      control_output.velocity[i] = 0;

      if (hasJoint(chain_[rod_arm_], current_state.name[i]))
      {
        control_output.position[i] = target_joint_positions_[rod_arm_](joint_index[rod_arm_]);
        // control_output.position[i] = joint_positions_[rod_arm](joint_index[rod_arm]) + commanded_joint_velocities[rod_arm](joint_index[rod_arm_])*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities[rod_arm_](joint_index[rod_arm_]);
        joint_index[rod_arm_]++;
      }

      if (hasJoint(chain_[surface_arm_], current_state.name[i]))
      {
        control_output.position[i] = target_joint_positions_[surface_arm_](joint_index[surface_arm_]);
        // control_output.position[i] = joint_positions_[surface_arm_](joint_index[surface_arm_]) + commanded_joint_velocities[surface_arm_](joint_index[surface_arm_])*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities[surface_arm_](joint_index[surface_arm_]);
        joint_index[surface_arm_]++;
      }
    }

    return control_output;
  }
}

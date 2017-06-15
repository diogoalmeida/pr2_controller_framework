#include <pr2_cartesian_controllers/mechanism_identification_controller.hpp>

namespace cartesian_controllers {

  MechanismIdentificationController::MechanismIdentificationController() : ControllerTemplate<pr2_cartesian_controllers::MechanismIdentificationAction,
                                                pr2_cartesian_controllers::MechanismIdentificationFeedback,
                                                pr2_cartesian_controllers::MechanismIdentificationResult>(), eef_to_grasp_(2)
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one folding action run
    startActionlib();
    finished_acquiring_goal_ = false;
    // pc_pub_ = nh_.advertise<visualization_msgs::Marker>("pc", 1);
    // p1_pub_ = nh_.advertise<visualization_msgs::Marker>("p1", 1);
    // p2_pub_ = nh_.advertise<visualization_msgs::Marker>("p2", 1);
    // r1_pub_ = nh_.advertise<visualization_msgs::Marker>("r1", 1);
    // r2_pub_ = nh_.advertise<visualization_msgs::Marker>("r2", 1);
    // wrench2_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("surface_frame_wrench", 1);
    feedback_thread_ = boost::thread(boost::bind(&MechanismIdentificationController::publishFeedback, this));
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
    boost::lock_guard<boost::mutex> guard(reference_mutex_);
    action_server_->setPreempted(result_);
    has_initial_ = false;
    ROS_WARN("Mechanism identification controller preempted!");
  }

  void MechanismIdentificationController::goalCB()
  {
    boost::shared_ptr<const pr2_cartesian_controllers::MechanismIdentificationGoal> goal = action_server_->acceptNewGoal();

    {
      boost::lock_guard<boost::mutex>guard(reference_mutex_);
      finished_acquiring_goal_ = false;
    }

    if(!ects_controller_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    if(!adaptive_controller_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    if(!estimator_.getParams(nh_))
    {
      action_server_->setAborted(result_);
      return;
    }

    rod_arm_ = goal->rod_arm;
    surface_arm_ = goal->surface_arm;
    goal_force_ = goal->goal_force;
    vd_amp_ = goal->vd_amplitude;
    vd_freq_ = goal->vd_frequency;
    wd_amp_ = goal->wd_amplitude;
    wd_freq_ = goal->wd_frequency;

    // TODO: Make method (class?)
    std::vector<double> linear_gains(3), ang_gains(3);
    for (int i = 0; i < 3; i++)
    {
      linear_gains[i] = comp_gains_[i];
      ang_gains[i] = comp_gains_[i + 3];
    }

    geometry_msgs::Vector3Stamped lin_gains_msg, ang_gains_msg;

    vectorStdToMsg(linear_gains, lin_gains_msg.vector);
    vectorStdToMsg(ang_gains, ang_gains_msg.vector);

    lin_gains_msg.header.frame_id = ft_frame_id_[surface_arm_];
    // ang_gains_msg.header.frame_id = ft_frame_id_[surface_arm_];

    geometry_msgs::PoseStamped pose_in, pose_out;
    try
    {
      lin_gains_msg.header.stamp = ros::Time(0);
      ang_gains_msg.header.stamp = ros::Time(0);
      listener_.transformVector(base_link_, lin_gains_msg, lin_gains_msg);
      // listener_.transformVector(base_link_, ang_gains_msg, ang_gains_msg);
      vectorMsgToStd(lin_gains_msg.vector, linear_gains);
      // vectorMsgToStd(ang_gains_msg.vector, ang_gains);
      Eigen::Matrix<double, 6, 1> gains;
      for (int i = 0; i < 3; i++)
      {
        gains[i] = linear_gains[i];
        gains[i + 3] = ang_gains[i];
      }

      twist_controller_.reset(new TwistController(gains));
      // end TODO

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
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
      action_server_->setAborted();
      return;
    }

    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      finished_acquiring_goal_ = true;
    }

    ROS_INFO("Folding controller server received a goal!");
  }

  void MechanismIdentificationController::publishFeedback()
  {
    // visualization_msgs::Marker pc_marker, p1_marker, p2_marker, r1_marker, r2_marker;
    // geometry_msgs::Vector3 r_vec;
    // geometry_msgs::WrenchStamped surface_wrench;
    // 
    // pc_marker.header.frame_id = chain_base_link_;
    // pc_marker.ns = std::string("folding");
    // pc_marker.type = pc_marker.SPHERE;
    // pc_marker.action = pc_marker.ADD;
    // pc_marker.scale.x = 0.01;
    // pc_marker.scale.y = 0.01;
    // pc_marker.scale.z = 0.01;
    // pc_marker.lifetime = ros::Duration(0);
    // pc_marker.frame_locked = false;
    // pc_marker.color.r = 1.0;
    // pc_marker.color.a = 1.0;
    // p1_marker = pc_marker;
    // p1_marker.id = 1;
    // p2_marker = pc_marker;
    // p2_marker.id = 2;
    // p1_marker.color.r = 0.0;
    // p2_marker.color.r = 0.0;
    // p1_marker.color.g = 1.0;
    // p2_marker.color.g = 1.0;
    // r1_marker = p1_marker;
    // r1_marker.id = 3;
    // r1_marker.scale.y = 0.005;
    // r1_marker.scale.z = 0.005;
    // r1_marker.type = r1_marker.ARROW;
    // r2_marker = r1_marker;
    // r2_marker.id = 4;
    // r2_marker.color = pc_marker.color;
    // 
    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive() && finished_acquiring_goal_)
        {
    //       boost::lock_guard<boost::mutex> guard(reference_mutex_);
    //       pc_marker.header.stamp = ros::Time::now();
    //       p1_marker.header.stamp = ros::Time::now();
    //       p2_marker.header.stamp = ros::Time::now();
    //       r1_marker.header.stamp = ros::Time::now();
    //       r2_marker.header.stamp = ros::Time::now();
    // 
    //       tf::poseEigenToMsg(pc_, pc_marker.pose);
    //       tf::poseEigenToMsg(p1_, p1_marker.pose);
    //       tf::poseEigenToMsg(p2_, p2_marker.pose);
    //       getMarkerPoints(p1_.translation(), pc_.translation(), r1_marker);
    //       getMarkerPoints(p2_.translation(), pc_.translation(), r2_marker);
    // 
    //       pc_pub_.publish(pc_marker);
    //       p1_pub_.publish(p1_marker);
    //       p2_pub_.publish(p2_marker);
    //       r1_pub_.publish(r1_marker);
    //       r2_pub_.publish(r2_marker);
    // 
    //       tf::vectorEigenToMsg(pc_.translation() - p1_.translation(), feedback_.r1);
    //       tf::vectorEigenToMsg(pc_.translation() - p2_.translation(), feedback_.r2);
    //       tf::vectorEigenToMsg(p1_.translation(), feedback_.p1);
    //       tf::vectorEigenToMsg(pc_.translation(), feedback_.pc);
    //       tf::vectorEigenToMsg(p2_.translation(), feedback_.p2);
    //       surface_wrench.header.frame_id = ft_frame_id_[surface_arm_];
    //       tf::wrenchEigenToMsg(wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]), surface_wrench.wrench);
    //       surface_wrench.header.stamp = ros::Time::now();
    //       feedback_.surface_wrench = surface_wrench;
    //       wrench2_pub_.publish(surface_wrench);
          feedback_.absolute_twist.header.stamp = ros::Time::now();
          feedback_.relative_twist.header.stamp = ros::Time::now();
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
    Eigen::Vector3d translational_dof_ground, rotational_dof_ground, p1, p2,
                    contact_force, contact_torque, surface_tangent_in_grasp, rotation_axis,
                    out_vel_lin, out_vel_ang; // all in the base_link frame
    Eigen::Matrix<double, 12, 1> ects_twist = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 14, 1> joint_commands;
    KDL::Twist comp_twist;
    KDL::Jacobian kdl_jac(7);

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
      return lastState(current_state);
    }

    if(!getChainJointState(current_state, chain_[rod_arm_], joint_positions_[rod_arm_], joint_velocities_[rod_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the rod arm. Aborting.");
      action_server_->setAborted();
      return lastState(current_state);
    }

    control_output = lastState(current_state, rod_arm_);

    for (int arm = 0; arm < 2; arm++) // Compute forward kinematics and convert to grasp frame
    {
      fkpos_[arm]->JntToCart(joint_positions_[arm], eef_kdl[arm]);
      fkvel_[arm]->JntToCart(joint_velocities_[arm], eef_vel_kdl[arm]);
      q_dot[arm] = joint_velocities_[arm].qdot.data;
      eef_grasp_kdl[arm] = eef_kdl[arm]*eef_to_grasp_[arm];
      eef_grasp_vel_kdl[arm] = eef_vel_kdl[arm]*eef_to_grasp_[arm];
      eef_twist[arm] = eef_grasp_vel_kdl[arm].GetTwist();
      tf::transformKDLToEigen(eef_to_grasp_[arm], eef_to_grasp_eig[arm]);
      tf::transformKDLToEigen(eef_grasp_kdl[arm], grasp_point_frame[arm]);
      tf::twistKDLToEigen(eef_twist[arm], eef_twist_eig[arm]);
      commanded_joint_velocities[arm] = KDL::JntArray(chain_[arm].getNrOfJoints());
      jac_solver_[arm]->JntToJac(joint_positions_[arm], kdl_jac);
      jacobian[arm] = kdl_jac.data;
    }

    rotational_dof_ground  = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,1);
    translational_dof_ground = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,0);
    p1 = grasp_point_frame[rod_arm_].translation();
    p2 = grasp_point_frame[surface_arm_].translation();
    contact_force = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(0,0);
    contact_torque = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(3,0);
    p1_ = grasp_point_frame[rod_arm_];
    p2_ = grasp_point_frame[surface_arm_];

    if (!has_initial_)
    {
      estimator_.initialize(Eigen::Vector3d::Zero());
      elapsed_ = ros::Time(0);
      has_initial_ = true;
    }

    elapsed_ += dt;

    if (use_estimates_)
    {
      // TODO
    }
    else
    {
      pc_.translation() = rod_length_*p1_.matrix().block<3,1>(0,0); // it is assumed that the rod is aligned with the x axis of the grasp frame
      ects_twist.block<3,1>(6,0) = vd_amp_*sin(2*M_PI*vd_freq_*elapsed_.toSec())*translational_dof_ground;
      ects_twist.block<3,1>(9,0) = wd_amp_*sin(2*M_PI*wd_freq_*elapsed_.toSec())*rotational_dof_ground;
    }
    pc_.linear() =  p1_.linear();
    
    tf::twistEigenToMsg(ects_twist.block<6,1>(0,0), feedback_.absolute_twist.twist);
    tf::twistEigenToMsg(ects_twist.block<6,1>(6,0), feedback_.relative_twist.twist);
    
    joint_commands = ects_controller_.control(jacobian[rod_arm_], jacobian[surface_arm_], pc_.translation() - p1, pc_.translation() - p2, q_dot[rod_arm_], q_dot[surface_arm_], ects_twist);

    for (unsigned int i = 0; i < 7; i++)
    {
      commanded_joint_velocities[rod_arm_](i) = joint_commands[i];
      commanded_joint_velocities[surface_arm_](i) = joint_commands[i + 7];
    }

    std::vector<int> joint_index(2, 0);
    for (unsigned long i = 0; i < current_state.name.size(); i++)
    {
      control_output.velocity[i] = 0;

      if (hasJoint(chain_[rod_arm_], current_state.name[i]))
      {
        control_output.position[i] = joint_positions_[rod_arm_](joint_index[rod_arm_]) + commanded_joint_velocities[rod_arm_](joint_index[rod_arm_])*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities[rod_arm_](joint_index[rod_arm_]);
        joint_index[rod_arm_]++;
      }

      if (hasJoint(chain_[surface_arm_], current_state.name[i]))
      {
        control_output.position[i] = joint_positions_[surface_arm_](joint_index[surface_arm_]) + commanded_joint_velocities[surface_arm_](joint_index[surface_arm_])*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities[surface_arm_](joint_index[surface_arm_]);
        joint_index[surface_arm_]++;
      }
    }

    return control_output;
  }
}

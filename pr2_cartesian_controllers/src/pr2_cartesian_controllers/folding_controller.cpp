#include <pr2_cartesian_controllers/folding_controller.hpp>

namespace cartesian_controllers {

  FoldingController::FoldingController() : eef_to_grasp_(2) , ControllerTemplate<pr2_cartesian_controllers::FoldingControllerAction,
                                                pr2_cartesian_controllers::FoldingControllerFeedback,
                                                pr2_cartesian_controllers::FoldingControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one folding action run
    startActionlib();
    finished_acquiring_goal_ = false;
    pc_pub_ = nh_.advertise<visualization_msgs::Marker>("pc", 1);
    p1_pub_ = nh_.advertise<visualization_msgs::Marker>("p1", 1);
    p2_pub_ = nh_.advertise<visualization_msgs::Marker>("p2", 1);
    r1_pub_ = nh_.advertise<visualization_msgs::Marker>("r1", 1);
    r2_pub_ = nh_.advertise<visualization_msgs::Marker>("r2", 1);
    wrench2_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("surface_frame_wrench", 1);
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

    {
      boost::lock_guard<boost::mutex>guard(reference_mutex_);
      finished_acquiring_goal_ = false;
    }

    if(!controller_.getParams(nh_))
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
    goal_p_ = goal->position_offset;
    goal_theta_ = goal->orientation_goal;
    goal_force_ = goal->force_goal;

    initTwistController(comp_gains_, base_link_, ft_frame_id_[surface_arm_]);
    geometry_msgs::PoseStamped pose_in, pose_out;

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

  void FoldingController::publishFeedback()
  {
    visualization_msgs::Marker pc_marker, p1_marker, p2_marker, r1_marker, r2_marker;
    geometry_msgs::Vector3 r_vec;
    geometry_msgs::WrenchStamped surface_wrench;

    pc_marker.header.frame_id = chain_base_link_;
    pc_marker.ns = std::string("folding");
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

          tf::poseEigenToMsg(pc_, pc_marker.pose);
          tf::poseEigenToMsg(p1_, p1_marker.pose);
          tf::poseEigenToMsg(p2_, p2_marker.pose);
          getMarkerPoints(p1_.translation(), pc_.translation(), r1_marker);
          getMarkerPoints(p2_.translation(), pc_.translation(), r2_marker);

          pc_pub_.publish(pc_marker);
          p1_pub_.publish(p1_marker);
          p2_pub_.publish(p2_marker);
          r1_pub_.publish(r1_marker);
          r2_pub_.publish(r2_marker);

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
    if (!getParam("/folding_controller/action_server_name", action_name_))
    {
      return false;
    }

    if (!getParam("/folding_controller/rotational_gains", comp_gains_))
    {
      return false;
    }

    if (!getParam("/folding_controller/use_estimates", use_estimates_))
    {
      return false;
    }

    if (!getParam("/folding_controller/rod_length", rod_length_))
    {
      return false;
    }

    return true;
  }

  sensor_msgs::JointState FoldingController::updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt)
  {
    sensor_msgs::JointState control_output = current_state;
    std::vector<KDL::Frame> eef_grasp_kdl(2), eef_kdl(2);
    std::vector<KDL::FrameVel> eef_vel_kdl(2), eef_grasp_vel_kdl(2);
    std::vector<KDL::Twist> eef_twist(2);
    std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2);
    std::vector<Eigen::Matrix<double, 6, 1> > eef_twist_eig(2);
    std::vector<KDL::Twist> command_twist(2);
    std::vector<KDL::JntArray> commanded_joint_velocities(2);
    Eigen::Vector3d surface_normal, surface_tangent, p1, p2, omega, pd, surface_normal_in_grasp,
                    contact_force, contact_torque, surface_tangent_in_grasp, rotation_axis, r,
                    out_vel_lin, out_vel_ang; // all in the base_link frame
    KDL::Twist comp_twist;

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
      eef_grasp_kdl[arm] = eef_kdl[arm]*eef_to_grasp_[arm];
      eef_grasp_vel_kdl[arm] = eef_vel_kdl[arm]*eef_to_grasp_[arm];
      eef_twist[arm] = eef_grasp_vel_kdl[arm].GetTwist();
      tf::transformKDLToEigen(eef_to_grasp_[arm], eef_to_grasp_eig[arm]);
      tf::transformKDLToEigen(eef_grasp_kdl[arm], grasp_point_frame[arm]);
      tf::twistKDLToEigen(eef_twist[arm], eef_twist_eig[arm]);
      commanded_joint_velocities[arm] = KDL::JntArray(chain_[arm].getNrOfJoints());
    }

    surface_normal = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,2);
    surface_tangent = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,0);
    surface_normal_in_grasp = eef_to_grasp_eig[surface_arm_].matrix().block<3,1>(0,2);
    surface_tangent_in_grasp = eef_to_grasp_eig[surface_arm_].matrix().block<3,1>(0,0);
    rotation_axis = grasp_point_frame[surface_arm_].matrix().block<3,1>(0,1);;
    p1 = grasp_point_frame[rod_arm_].translation();
    p2 = grasp_point_frame[surface_arm_].translation();
    contact_force = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(0,0);
    contact_torque = wrenchInFrame(surface_arm_, ft_frame_id_[surface_arm_]).block<3,1>(3,0);
    omega << eef_twist_eig[surface_arm_].block<3,1>(3,0);
    pd = grasp_point_frame[surface_arm_].translation() + goal_p_*surface_tangent;

    if (!has_initial_)
    {
      estimator_.initialize(Eigen::Vector3d::Zero());
      has_initial_ = true;
    }

    if (use_estimates_)
    {
      r = estimator_.estimate(Eigen::Vector3d::Zero(), contact_force, contact_torque, dt.toSec()); // expressed in the tool frame
      r = grasp_point_frame[surface_arm_].linear()*r;
    }
    else
    {
      r = rod_length_*grasp_point_frame[rod_arm_].matrix().block<3,1>(0,0); // it is assumed that the rod is aligned with the x axis of the grasp frame
    }

    p1_ = grasp_point_frame[rod_arm_];
    p2_ = grasp_point_frame[surface_arm_];
    pc_.translation() = r + p2;
    pc_.linear() =  Eigen::Matrix<double, 3, 3>::Identity();
    controller_.control(pd, goal_theta_, goal_force_, surface_tangent, surface_normal, r, p1, contact_force, out_vel_lin, out_vel_ang, dt.toSec());

    eef_twist_eig[rod_arm_] << out_vel_lin, out_vel_ang;

    tf::twistEigenToKDL(eef_twist_eig[rod_arm_], command_twist[rod_arm_]);
    tf::twistEigenToMsg(eef_twist_eig[rod_arm_], feedback_.controller_output.twist);

    comp_twist = twist_controller_->computeError(eef_grasp_kdl[rod_arm_], eef_grasp_kdl[surface_arm_]); // want to stay aligned with the surface_arm
    command_twist[rod_arm_] += comp_twist.RefPoint(eef_to_grasp_[rod_arm_].p);

    ikvel_[rod_arm_]->CartToJnt(joint_positions_[rod_arm_], command_twist[rod_arm_], commanded_joint_velocities[rod_arm_]);

    int joint_index = 0;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      control_output.velocity[i] = 0;

      if (hasJoint(chain_[rod_arm_], current_state.name[i]))
      {
        control_output.position[i] = joint_positions_[rod_arm_](joint_index) + commanded_joint_velocities[rod_arm_](joint_index)*dt.toSec();
        control_output.velocity[i] = commanded_joint_velocities[rod_arm_](joint_index);
        joint_index++;
      }
    }

    return control_output;
  }
}

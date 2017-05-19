#include <pr2_cartesian_controllers/folding_controller.hpp>

namespace cartesian_controllers {

  FoldingController::FoldingController() : eef_to_grasp(2) , ControllerTemplate<pr2_cartesian_controllers::FoldingControllerAction,
                                                pr2_cartesian_controllers::FoldingControllerFeedback,
                                                pr2_cartesian_controllers::FoldingControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    finished_acquiring_goal_ = false;
    pc_publisher_ = nh_.advertise<visualization_msgs::Marker>("contact_point_estimate", 1);
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

    {
      boost::lock_guard<boost::mutex> guard(reference_mutex_);
      finished_acquiring_goal_ = true;
    }

    ROS_INFO("Folding controller server received a goal!");
  }

  void FoldingController::publishFeedback()
  {
    visualization_msgs::Marker contact_point;

    try
    {
      while(ros::ok())
      {
        if (action_server_->isActive())
        {
          contact_point.header.frame_id = base_link_;
          contact_point.header.stamp = ros::Time::now();
          contact_point.ns = std::string("folding");
          contact_point.type = contact_point.SPHERE;
          contact_point.action = contact_point.ADD;
          tf::poseEigenToMsg(pc_, contact_point.pose);
          contact_point.scale.x = 0.01;
          contact_point.scale.y = 0.01;
          contact_point.scale.z = 0.01;
          contact_point.lifetime = ros::Duration(0);
          contact_point.frame_locked = false;
          contact_point.color.r = 1.0;
          contact_point.color.a = 1.0;
          pc_publisher_.publish(contact_point);
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

    // if (!getParam("/folding_controller/rotational_gains", rot_gains_))
    // {
    //   return false;
    // }

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
    Eigen::Vector3d surface_normal, surface_tangent, p1, omega, pd, surface_normal_in_grasp,
                    contact_force, contact_torque, surface_tangent_in_grasp, rotation_axis, r,
                    out_vel_lin, out_vel_ang; // all in the base_link frame

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
      lastState(current_state);
    }

    if(!getChainJointState(current_state, chain_[rod_arm_], joint_positions_[rod_arm_], joint_velocities_[rod_arm_]))
    {
      ROS_ERROR("Failed to get the chain joint state for the rod arm. Aborting.");
      action_server_->setAborted();
      lastState(current_state);
    }

    for (int arm = 0; arm < 2; arm++) // Compute forward kinematics and convert to grasp frame
    {
      fkpos_[arm]->JntToCart(joint_positions_[arm], eef_kdl[arm]);
      fkvel_[arm]->JntToCart(joint_velocities_[arm], eef_vel_kdl[arm]);
      eef_grasp_kdl[arm] = eef_kdl[arm]*eef_to_grasp[arm];
      eef_grasp_vel_kdl[arm] = eef_vel_kdl[arm]*eef_to_grasp[arm];
      eef_twist[arm] = eef_grasp_vel_kdl[arm].GetTwist();
      tf::transformKDLToEigen(eef_to_grasp[arm], eef_to_grasp_eig[arm]);
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
    contact_force = wrenchInFrame(surface_arm_, base_link_).block<3,1>(0,0);
    contact_torque = wrenchInFrame(surface_arm_, base_link_).block<3,1>(3,0);
    omega << eef_twist_eig[rod_arm_].block<3,1>(3,0);
    pd = grasp_point_frame[surface_arm_].translation() + goal_p_*surface_tangent;

    if (!has_initial_)
    {
      estimator_.initialize(pd - p1);
      has_initial_ = true;
    }

    if (use_estimates_)
    {
      r = estimator_.estimate(omega, contact_force, contact_torque, dt.toSec());
    }
    else
    {
      r = rod_length_*grasp_point_frame[rod_arm_].matrix().block<3,1>(0,0); // it is assumed that the rod is aligned with the x axis of the grasp frame
    }

    pc_.translation() = r + p1;
    pc_.linear() =  Eigen::Matrix<double, 3, 3>::Identity();
    controller_.control(pd, goal_theta_, goal_force_, surface_tangent, surface_normal, r, p1, contact_force, out_vel_lin, out_vel_ang, dt.toSec());

    eef_twist_eig[rod_arm_] << out_vel_lin, out_vel_ang;
    tf::twistEigenToKDL(eef_twist_eig[rod_arm_], command_twist[rod_arm_]);
    tf::twistEigenToMsg(eef_twist_eig[rod_arm_], feedback_.controller_output.twist);

    ikvel_[rod_arm_]->CartToJnt(joint_positions_[rod_arm_], command_twist[rod_arm_], commanded_joint_velocities[rod_arm_]);

    int joint_index = 0;
    for (int i = 0; i < current_state.name.size(); i++)
    {
      control_output.velocity[i] = 0;
      control_output.effort[i] = 0;

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

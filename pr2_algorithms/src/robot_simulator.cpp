#include <pr2_algorithms/robot_simulator.hpp>

RobotSimulator::RobotSimulator(double frequency = 10)
{
  urdf::Model model;

  if(!model.initParam("/robot_description")){
      ROS_ERROR("ERROR getting robot description (/robot_description)");
      return;
  }

  ROS_INFO("Loading URDF model");
  kdl_parser::treeFromUrdfModel(model, tree_);
  robot_publisher_.reset(new robot_state_publisher::RobotStatePublisher(tree_));
  for(auto const &element : tree_.getSegments()) // Initialize the joints
  {
    KDL::Joint::JointType type = element.second.segment.getJoint().getType();

    if (type != KDL::Joint::JointType::None)
    {
      joint_positions_[element.second.segment.getJoint().getName()] = 0.0;
    }
  }

  robot_publisher_->publishTransforms(joint_positions_, ros::Time::now(), "");
  robot_publisher_->publishFixedTransforms("");
  frequency_ = frequency;

  sim_thread_ = boost::thread(&RobotSimulator::simulation, this);
}

KDL::Frame RobotSimulator::getKDLPose(const std::vector<double> &pose_in)
{
  tf::Quaternion quat(pose_in[3], pose_in[4], pose_in[5], pose_in[6]);
  tf::Matrix3x3 mat(quat);
  KDL::Vector vec(pose_in[0], pose_in[1], pose_in[2]);
  KDL::Rotation rot;
  double roll, pitch, yaw;

  mat.getRPY(roll, pitch, yaw);
  rot = rot.RPY(roll, pitch, yaw);

  KDL::Frame frame(rot, vec);

  return frame;
}

bool RobotSimulator::getKinematicChain(const std::string &end_effector_link, KDL::Chain &chain)
{
  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      chain = chain_[i];
      return true;
    }
  }

  ROS_ERROR("Could not find kinematic chain for end-effector %s. Did you initialize?", end_effector_link.c_str());
  return false;
}

bool RobotSimulator::getPose(const std::string &end_effector_link, KDL::Frame &pose)
{
  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      fk_solver_[i]->JntToCart(joint_state_[i].q, pose);
      return true;
    }
  }

  ROS_ERROR("Could not find end-effector %s's pose. Did you initialize the kinematic chain?", end_effector_link.c_str());
  return false;
}

bool RobotSimulator::getJointState(const std::string &end_effector_link, KDL::JntArray &q)
{
  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      q = joint_state_[i].q;
      return true;
    }
  }

  ROS_ERROR("Could not find end-effector %s's joint state. Did you initialize the kinematic chain?", end_effector_link.c_str());
  return false;
}

bool RobotSimulator::initKinematicChain(const std::string &base_link, const std::string &end_effector_link, const std::vector<double> &desired_pose)
{
  KDL::JntArrayVel joint_state;
  KDL::Chain chain;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
  KDL::Frame pose_frame;
  boost::mutex::scoped_lock lock(velocities_mutex_);

  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      ROS_ERROR("Tried to add an already initialized kinematic chain %s", end_effector_link.c_str());
      return false;
    }
  }

  end_effector_link_name_.push_back(end_effector_link);

  if(!tree_.getChain(base_link, end_effector_link, chain))
  {
    ROS_ERROR("Failed to get the kinematic chain from %s to %s", base_link.c_str(), end_effector_link.c_str());
  }
  joint_state.q.resize(chain.getNrOfJoints());
  joint_state.qdot.resize(chain.getNrOfJoints());
  Eigen::VectorXd velocities(chain.getNrOfJoints());
  velocities = Eigen::VectorXd::Zero(chain.getNrOfJoints());
  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  ik_solver.reset(new KDL::ChainIkSolverPos_LMA(chain));
  pose_frame = getKDLPose(desired_pose);

  for (int i = 0; i < 7; i++)
  {
    joint_state.q(i) = desired_pose[i];
  }
  // 
  // ik_solver->CartToJnt(joint_state.q, pose_frame, joint_state.q);
  // 
  // ROS_INFO("Initial joint config:");
  // for(int i = 0; i < 7; i++)
  // {
  //   std::cout << joint_state.q(i) << " ";
  // }
  // std::cout << std::endl;

  chain_.push_back(chain);
  joint_state_.push_back(joint_state);
  fk_solver_.push_back(fk_solver);
  ik_solver_.push_back(ik_solver);
  current_joint_velocities_.push_back(velocities);

  return true;
}

bool RobotSimulator::setJointVelocities(const std::string &end_effector_link, const Eigen::VectorXd &joint_velocities)
{
  boost::mutex::scoped_lock lock(velocities_mutex_);
  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      current_joint_velocities_[i] = joint_velocities;
      return true;
    }
  }

  ROS_ERROR("Could not find end-effector %s's joint velocities. Did you initialize the kinematic chain?", end_effector_link.c_str());
  return false;
}

bool RobotSimulator::applyJointVelocities(const Eigen::VectorXd &joint_velocities, const std::string &end_effector_link, double dt)
{
  int arm = -1;

  for (int i = 0; i < end_effector_link_name_.size(); i++)
  {
    if (end_effector_link_name_[i] == end_effector_link)
    {
      arm = i;
      break;
    }
  }

  if (arm == -1)
  {
    ROS_ERROR("Could not find kinematic chain for end-effector %s. Did you initialize?", end_effector_link.c_str());
    return false;
  }

  if (joint_velocities.rows() != chain_[arm].getNrOfJoints())
  {
    ROS_ERROR("Got %lu joint velocity values, but kinematic chain %s expects %u!", joint_velocities.rows(), end_effector_link.c_str(), chain_[arm].getNrOfJoints());
    return false;
  }

  for (int i = 0; i < joint_velocities.rows(); i++)
  {
    joint_state_[arm].qdot(i) = joint_velocities[i];
    joint_state_[arm].q(i) = joint_state_[arm].q(i) + joint_velocities[i]*dt;
    joint_positions_[chain_[arm].getSegment(i).getJoint().getName()] = joint_state_[arm].q(i);
  }

  robot_publisher_->publishTransforms(joint_positions_, ros::Time::now(), "");
  robot_publisher_->publishFixedTransforms("");
  return true;
}

void RobotSimulator::simulation()
{
  ros::Rate r(frequency_);
  ros::Time prev = ros::Time::now();
  ros::Duration elapsed;
  while(ros::ok())
  {
      elapsed = ros::Time::now() - prev;
    {
      boost::mutex::scoped_lock lock(velocities_mutex_);
      for (int i = 0; i < end_effector_link_name_.size(); i++)
      {
        applyJointVelocities(current_joint_velocities_[i], end_effector_link_name_[i], elapsed.toSec());
      }
    }
    prev = ros::Time::now();
    r.sleep();
  }
}
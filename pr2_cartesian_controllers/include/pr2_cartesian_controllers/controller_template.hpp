#ifndef __CONTROLLER_TEMPLATE__
#define __CONTROLLER_TEMPLATE__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
// #include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/TwistController.hpp>
#define NUM_ARMS 2

namespace cartesian_controllers{

/**
  Defines the basic cartesian controller interface to be instantiated in the
  joint controller level.
**/
class ControllerBase
{
public:
  ControllerBase(){}
  virtual ~ControllerBase(){}

  /**
    Method for computing the desired joint states given the control algorithm.

    @param current_state Current joint states.
    @param dt Elapsed time since last control loop.

    @return Desired joint states.
  **/
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt) = 0;
};

/**
  Defines the interface for all the cartesian controllers, allowing for
  an easier embedding in the PR2 realtime loop.
**/
template <class ActionClass, class ActionFeedback, class ActionResult>
class ControllerTemplate : public ControllerBase
{
public:
  ControllerTemplate();
  virtual ~ControllerTemplate()
  {
  }

protected:
  /**
    Return the last controlled joint state. If the controller does not have
    an active actionlib goal, it will set the references of the joint controller
    to the last desired position (and null velocity).

    @param current The current joint state.
    @return The last commanded joint state before the actionlib goal was
    preempted or completed.
  **/
  sensor_msgs::JointState lastState(const sensor_msgs::JointState &current);

  /**
    Return the last controlled joint state for the joints that are not in the given arm.
    If the controller does not have an active actionlib goal, it will set the references of the joint controller
    to the last desired position (and null velocity). The current joint state will only be
    changed for the given arm.

    @param current The current joint state.
    @param arm The index of the joint chain to be considered.
    @return The last commanded joint state before the actionlib goal was
    preempted or completed.
  **/
  sensor_msgs::JointState lastState(const sensor_msgs::JointState &current, int arm);

  /**
    Initialize the kinematic chain and joint arrays for an arm defined by its end-effector link.
    The kinematic chain is assumed to start at chain_base_link_.

    @param end_effector_link The final link of the kinematic chain.
    @param chain The kinematic chain to be initialized.
    @param joint_positions The joint positions array of the kinematic chain.
    @param joint_velocities The joint velocities array of the kinematic chain.
    @param actuated_joint_names The list of joint names of the kinematic chain.
  **/
  void initializeArm(std::string end_effector_link, KDL::Chain &chain, KDL::JntArray &joint_positions, KDL::JntArrayVel &joint_velocities, std::vector<std::string> &actuated_joint_names);

  /**
    Initialize the kinematic solvers to be used with a kinematic chain.

    @param The chain for which we want to initialized the solvers.
    @param fkpos Positional forward kinematics solver.
    @param fkvel Velocity forward kinematics solver.
    @param ikpos Positional inverse kinematics solver.
    @param ikvel Velocity inverse kinematics solver.
    @param jac_solver The jacobian solver.
  **/
  void initializeSolvers(const KDL::Chain &chain, boost::shared_ptr<KDL::ChainFkSolverPos_recursive> &fkpos, boost::shared_ptr<KDL::ChainFkSolverVel_recursive> &fkvel, boost::shared_ptr<KDL::ChainIkSolverVel_pinv_nso> &ikvel, boost::shared_ptr<KDL::ChainIkSolverPos_LMA> &ikpos, boost::shared_ptr<KDL::ChainJntToJacSolver> &jac_solver);

  /**
    Initializes the wrench vector, subscriber and publisher for a given wrench topic.

    @param measured_wrench The eigen vector where the six-dimensional wrench is going to be stored.
    @param ft_sub The ros subscriber for the sensor.
    @param ft_pub The ros publisher that allows for modified measurements to be published. The ros publisher will publish in ft_topic_name/corrected.
    @param ft_topic_name The ros topic where the original wrench measurements are published.
  **/
  void initializeWrenchComms(Eigen::Matrix<double, 6, 1> &measured_wrench, ros::Subscriber &ft_sub, ros::Publisher &ft_pub, std::string ft_topic_name);

  /**
    Initializes the twist controller by translating the desired gains from the given frame id to the base link.

    @param comp_gains The gains in frame_id.
    @param base_link The base link name.
    @param frame_id The frame where the gains are expressed.
  **/
  void initTwistController(const std::vector<double> &comp_gains, const std::string &base_link, const std::string &frame_id);

  /**
    Provides access to the measured wrench in a given frame.

    @param arm_index The index of the arm measuring the wrench.
    @param frame The desired wrench frame.
    @return The wrench in the given frame
  **/
  Eigen::Matrix<double, 6, 1> wrenchInFrame(int arm_index, const std::string &frame);

  /**
    Goal callback method to be implemented in the cartesian controllers.
  **/
  virtual void goalCB() = 0;

  /**
    Preempt callback method to be implemented in the cartesian controllers.
  **/
  virtual void preemptCB() = 0;

  /**
    Method that manages the starting of the actionlib server of each cartesian
    controller.
  **/
  void startActionlib();

  /**
    Load the parameters that are common to all the cartesian controllers.

    @return False if an error occurs, True otherwise.
  **/
  bool loadGenericParams();

  /**
    Method to be implemented in the cartesian controllers that loads controller
    specific parameters.
  **/
  virtual bool loadParams() = 0;

  /**
    Obtains wrench measurments for a force torque sensor.

    @param msg The force torque message from the sensor node.
  **/
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  /**
    Gets the actuated joint limits from the URDF description of the robot
    for a given kinematic chain.

    @param chain The kinematic chain for which the limits are going to be found.
    @param min_limits The minimum position limits of the joint.
    @param max_limits The maximum position limits of the joint.
  **/
  void getJointLimits(const KDL::Chain &chain, KDL::JntArray &min_limits, KDL::JntArray &max_limits);

  /**
    Check if a chain has the given joint_name.

    @param chain The kinematic chain where to look for the joint.
    @param joint_name The joint name we wish to check.
  **/
  bool hasJoint(const KDL::Chain &chain, const std::string &joint_name);

  /**
    Fills in the joint arrays with the state of a given kinematic chain.
    The joint state might include joints outside of the kinematic chain, so there is
    the need to process it.

    @param current_state The robot joint state.
    @param chain The target kinematic chain.
    @param positions The joint positions of the kinematic chain.
    @param velocities The joint velocities of the kinematic chain.
    @return True if the full joint chain was found in the current state, false otherwise.
  **/
  bool getChainJointState(const sensor_msgs::JointState &current_state, const KDL::Chain &chain, KDL::JntArray &positions, KDL::JntArrayVel &velocities);

  /**
    Get the value for indexing the joint positions for the given joint name.

    @param joint_names The vector with the available joint names.
    @param name The name of the query joint.
    @return the index of the queried joint in the joint names vector.
    @throws logic_error if the name does not exist in the joint_names vector.
  **/
  int getJointIndex(const std::vector<std::string> &joint_names, const std::string &name);

  /**
    Fills a marker with the given initial and end point. Clears existing points.

    @param initial_point Initial marker point.
    @param final_point Final marker point.
    @param marker The marker object.
  **/
  void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker);

  /**
    Convert a geometry msgs vector to an std vector.

    @param in The geometry msgs vector.
    @param out The converted std vector.
  **/
  void vectorMsgToStd(const geometry_msgs::Vector3 &in, std::vector<double> &out);

  /**
    Convert an std vector to a geometry msgs vector.

    @param in The std vector.
    @param out The converted geometry msgs vector.
  **/
  void vectorStdToMsg(const std::vector<double> &in, geometry_msgs::Vector3 &out);

  /**
    Wraps the ROS NodeHandle getParam method with an error message.

    @param param_name The name of the parameter address in the parameter server.
    @param var The variable where to store the parameter.
  **/
  bool getParam(const std::string &param_name, std::string &var);
  bool getParam(const std::string &param_name, double &var);
  bool getParam(const std::string &param_name, std::vector<double> &var);
  bool getParam(const std::string &param_name, int &var);
  bool getParam(const std::string &param_name, bool &var);

protected:
  // Robot related
  sensor_msgs::JointState robot_state;
  std::vector<KDL::JntArray> joint_positions_;
  std::vector<KDL::JntArrayVel> joint_velocities_;
  std::vector<std::vector<std::string> > actuated_joint_names_; // list of actuated joints per arm

  // KDL::ChainIkSolverVel_wdls *ikvel_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverVel_pinv_nso> > ikvel_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos_LMA> > ikpos_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos_recursive> > fkpos_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverVel_recursive> > fkvel_;
  std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver_;
  std::vector<KDL::Chain> chain_;
  urdf::Model model_;
  std::vector<std::string> end_effector_link_;
  std::vector<std::string> ft_topic_name_;
  std::vector<std::string> ft_frame_id_;
  std::vector<std::string> ft_sensor_frame_;
  std::string base_link_, chain_base_link_;
  double eps_; // ikSolverVel epsilon
  double alpha_; // ikSolverVel alpha
  int maxiter_; // ikSolverVel maxiter
  double nso_weights_;
  double feedback_hz_;
  bool has_state_;
  sensor_msgs::JointState last_state_;

  //Actionlib
  boost::shared_ptr<actionlib::SimpleActionServer<ActionClass> > action_server_;
  ActionFeedback feedback_;
  ActionResult result_;
  std::string action_name_;

  boost::thread feedback_thread_;
  boost::mutex reference_mutex_;

  // ROS
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> ft_sub_;
  std::vector<ros::Publisher> ft_pub_;
  tf::TransformListener listener_;

  std::vector<Eigen::Matrix<double, 6, 1> > measured_wrench_;
  double force_d_;
  boost::shared_ptr<TwistController> twist_controller_;

private:
  tf::TransformBroadcaster broadcaster_;
};

// Implementing the template class in its header file saves some headaches
// later on: http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor

template <class ActionClass, class ActionFeedback, class ActionResult>
ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::ControllerTemplate() :
 joint_positions_(NUM_ARMS),
 joint_velocities_(NUM_ARMS),
 ikvel_(NUM_ARMS),
 ikpos_(NUM_ARMS),
 fkpos_(NUM_ARMS),
 fkvel_(NUM_ARMS),
 jac_solver_(NUM_ARMS),
 chain_(NUM_ARMS),
 end_effector_link_(NUM_ARMS),
 ft_topic_name_(NUM_ARMS),
 ft_frame_id_(NUM_ARMS),
 ft_sensor_frame_(NUM_ARMS),
 ft_sub_(NUM_ARMS),
 ft_pub_(NUM_ARMS),
 actuated_joint_names_(NUM_ARMS),
 measured_wrench_(NUM_ARMS)
{
  nh_ = ros::NodeHandle("~");

  if(!loadGenericParams())
  {
    ros::shutdown();
    exit(0);
  }

  for (int i = 0; i < NUM_ARMS; i++)
  {
    initializeArm(end_effector_link_[i], chain_[i], joint_positions_[i], joint_velocities_[i], actuated_joint_names_[i]);
    initializeSolvers(chain_[i], fkpos_[i], fkvel_[i], ikvel_[i], ikpos_[i], jac_solver_[i]);
    initializeWrenchComms(measured_wrench_[i], ft_sub_[i], ft_pub_[i], ft_topic_name_[i]);
  }

  has_state_ = false;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::initializeArm(std::string end_effector_link, KDL::Chain &chain, KDL::JntArray &joint_positions, KDL::JntArrayVel &joint_velocities, std::vector<std::string> &actuated_joint_names)
{
  KDL::Tree tree;
  KDL::Joint kdl_joint;
  kdl_parser::treeFromUrdfModel(model_, tree); // convert URDF description of the robot into a KDL tree
  tree.getChain(chain_base_link_, end_effector_link, chain);
  joint_positions.resize(chain.getNrOfJoints());
  joint_velocities.q.resize(chain.getNrOfJoints());
  joint_velocities.qdot.resize(chain.getNrOfJoints());

  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    kdl_joint = chain.getSegment(i).getJoint();

    if (kdl_joint.getTypeName() == "None")
    {
      continue;
    }

    actuated_joint_names.push_back(kdl_joint.getName());
  }
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::initializeSolvers(const KDL::Chain &chain, boost::shared_ptr<KDL::ChainFkSolverPos_recursive> &fkpos, boost::shared_ptr<KDL::ChainFkSolverVel_recursive> &fkvel, boost::shared_ptr<KDL::ChainIkSolverVel_pinv_nso> &ikvel, boost::shared_ptr<KDL::ChainIkSolverPos_LMA> &ikpos, boost::shared_ptr<KDL::ChainJntToJacSolver> &jac_solver)
{
  KDL::JntArray min_limits, max_limits;
  KDL::JntArray optimal_values, weights;

  getJointLimits(chain, min_limits, max_limits);
  ROS_DEBUG("Min limits rows: %d, min limits columns: %d", min_limits.rows(), min_limits.columns());
  optimal_values.resize(chain.getNrOfJoints());
  weights.resize(chain.getNrOfJoints());

  ROS_DEBUG("Joint limits: ");
  for (unsigned int i = 0; i < chain.getNrOfJoints(); i++) // define the optimal joint values as the one that's as far away from joint limits as possible
  {
    optimal_values(i) = (min_limits(i) + max_limits(i))/2;

    ROS_DEBUG("Joint: %d, min_limit: %.2f, max_limit: %.2f, optimal_value: %.2f", i, min_limits(i), max_limits(i), optimal_values(i));

    if (min_limits(i) == max_limits(i)) // Do not weight in joints with no limits in the nullspace optimization method
    {
      weights(i) = 0;
    }
    else
    {
      weights(i) = nso_weights_;
    }

    ROS_DEBUG("Weight: %.2f\n\n", weights(i));
  }


  fkpos.reset(new KDL::ChainFkSolverPos_recursive(chain));
  fkvel.reset(new KDL::ChainFkSolverVel_recursive(chain));
  // ikvel = new KDL::ChainIkSolverVel_wdls(chain, eps_);
  // ikvel = new KDL::ChainIkSolverVel_pinv_nso(chain, eps_);
  ikvel.reset(new KDL::ChainIkSolverVel_pinv_nso(chain, optimal_values, weights, eps_, maxiter_, alpha_));
  ikpos.reset(new KDL::ChainIkSolverPos_LMA(chain));
  jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getJointLimits(const KDL::Chain &chain, KDL::JntArray &min_limits, KDL::JntArray &max_limits)
{
  KDL::Joint kdl_joint;
  boost::shared_ptr<const urdf::Joint> urdf_joint;
  int j = 0;

  min_limits.resize(chain.getNrOfJoints());
  max_limits.resize(chain.getNrOfJoints());

  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) // get joint limits
  {
    kdl_joint = chain.getSegment(i).getJoint();

    if (kdl_joint.getTypeName() == "None")
    {
      continue;
    }

    urdf_joint = model_.getJoint(kdl_joint.getName());
    min_limits(j) = urdf_joint->limits->lower;
    max_limits(j) = urdf_joint->limits->upper;
    j++;
  }
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getChainJointState(const sensor_msgs::JointState &current_state, const KDL::Chain &chain, KDL::JntArray &positions, KDL::JntArrayVel &velocities)
{
  unsigned int processed_joints = 0;
  for (unsigned long i = 0; i < current_state.name.size(); i++)
  {
    if (hasJoint(chain, current_state.name[i]))
    {
      positions(processed_joints) = current_state.position[i];
      velocities.q(processed_joints) = current_state.position[i];
      velocities.qdot(processed_joints) = current_state.velocity[i];
      processed_joints++;
    }
  }

  if (processed_joints != chain.getNrOfJoints())
  {
    ROS_ERROR("Failed to acquire chain joint state");
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::initializeWrenchComms(Eigen::Matrix<double, 6, 1> &measured_wrench, ros::Subscriber &ft_sub, ros::Publisher &ft_pub, std::string ft_topic_name)
{
  // Subscribe to force and torque measurements
  measured_wrench << 0, 0, 0, 0, 0, 0;
  ft_sub = nh_.subscribe(ft_topic_name, 1, &ControllerTemplate::forceTorqueCB, this); // we will pass the topic name to the subscriber to allow the proper wrench vector to be filled.
  ft_pub = nh_.advertise<geometry_msgs::WrenchStamped>(ft_topic_name + "/converted", 1);
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::initTwistController(const std::vector<double> &comp_gains, const std::string &base_link, const std::string &frame_id)
{
  std::vector<double> linear_gains(3), ang_gains(3);
  for (int i = 0; i < 3; i++)
  {
    linear_gains[i] = comp_gains[i];
    ang_gains[i] = comp_gains[i + 3];
  }

  geometry_msgs::Vector3Stamped lin_gains_msg, ang_gains_msg;

  vectorStdToMsg(linear_gains, lin_gains_msg.vector);
  vectorStdToMsg(ang_gains, ang_gains_msg.vector);

  lin_gains_msg.header.frame_id = frame_id;
  // ang_gains_msg.header.frame_id = ft_frame_id_[surface_arm_];

  try
  {
    lin_gains_msg.header.stamp = ros::Time(0);
    ang_gains_msg.header.stamp = ros::Time(0);
    listener_.transformVector(base_link, lin_gains_msg, lin_gains_msg);
    vectorMsgToStd(lin_gains_msg.vector, linear_gains);
    Eigen::Matrix<double, 6, 1> gains;
    for (int i = 0; i < 3; i++)
    {
      gains[i] = linear_gains[i];
      gains[i + 3] = ang_gains[i];
    }

    twist_controller_.reset(new TwistController(gains));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
    action_server_->setAborted();
    return;
  }
}

template <class ActionClass, class ActionFeedback, class ActionResult>
sensor_msgs::JointState ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::lastState(const sensor_msgs::JointState &current, int arm)
{
  sensor_msgs::JointState temp_state;

  temp_state = last_state_;

  for (unsigned long j = 0; j < temp_state.velocity.size(); j++)
  {
    if (hasJoint(chain_[arm], temp_state.name[j]))
    {
      temp_state.position[j] = current.position[j];
      temp_state.velocity[j] = current.velocity[j];
    }
  }

  return temp_state;
}


template <class ActionClass, class ActionFeedback, class ActionResult>
sensor_msgs::JointState ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::lastState(const sensor_msgs::JointState &current)
{
  if(!has_state_)
  {
    last_state_ = current;
    for (unsigned long i = 0; i < last_state_.velocity.size(); i++)
    {
      last_state_.velocity[i] = 0.0;
    }

    has_state_ = true;
  }

  return last_state_;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  KDL::Wrench wrench_kdl;
  KDL::Frame sensor_to_grasp_frame_kdl, sensor_frame_kdl, desired_kdl;
  geometry_msgs::PoseStamped sensor_to_grasp_frame, sensor_frame, desired;
  geometry_msgs::WrenchStamped converted_wrench;
  tf::Transform converted_wrench_frame;
  int sensor_num = -1;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  tf::wrenchMsgToKDL(msg->wrench, wrench_kdl);

  for (int i = 0; i < NUM_ARMS; i++)
  {
    if (msg->header.frame_id == ft_sensor_frame_[i])
    {
      sensor_num = i;
      break;
    }
  }

  if (sensor_num == -1)
  {
    ROS_ERROR("Got wrench message from sensor %s, which was not defined in the config file.", msg->header.frame_id.c_str());
    return;
  }

  converted_wrench = *msg;
  sensor_to_grasp_frame.header.frame_id = msg->header.frame_id;
  sensor_to_grasp_frame.header.stamp = ros::Time(0); // To enable transform with the latest ft data available
  sensor_to_grasp_frame.pose.position.x = 0;
  sensor_to_grasp_frame.pose.position.y = 0;
  sensor_to_grasp_frame.pose.position.z = 0;
  sensor_to_grasp_frame.pose.orientation.x = 0;
  sensor_to_grasp_frame.pose.orientation.y = 0;
  sensor_to_grasp_frame.pose.orientation.z = 0;
  sensor_to_grasp_frame.pose.orientation.w = 1;
  converted_wrench.header.frame_id = ft_frame_id_[sensor_num];

  try
  {
    // obtain a vector from the wrench frame id to the desired ft frame
    listener_.transformPose(ft_frame_id_[sensor_num], sensor_to_grasp_frame, sensor_to_grasp_frame);
    // listener_.transformPose(base_link_, sensor_frame, sensor_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
  }

  tf::poseMsgToKDL(sensor_to_grasp_frame.pose, sensor_to_grasp_frame_kdl);
  wrench_kdl = sensor_to_grasp_frame_kdl*wrench_kdl;
  tf::wrenchKDLToMsg(wrench_kdl, converted_wrench.wrench);
  tf::wrenchKDLToEigen(wrench_kdl, measured_wrench_[sensor_num]);
  ft_pub_[sensor_num].publish(converted_wrench);

  // HACK: PR2 had a broken force torque sensor. To allow dual-arm operations to
  // work smoothly, I'm doing this =x
  measured_wrench_[0] = measured_wrench_[sensor_num];
  measured_wrench_[1] = measured_wrench_[sensor_num];
}

template <class ActionClass, class ActionFeedback, class ActionResult>
Eigen::Matrix<double, 6, 1> ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::wrenchInFrame(int arm_index, const std::string &frame)
{
  KDL::Wrench wrench_kdl;
  geometry_msgs::PoseStamped sensor_to_desired_frame, desired_frame_to_base;
  KDL::Frame sensor_to_desired_frame_kdl, desired_frame_to_base_kdl;
  Eigen::Matrix<double, 6, 1> converted_wrench;

  // HACK: Always use arm 0, as arm 1 has a broken sensor
  tf::wrenchEigenToKDL(measured_wrench_[0], wrench_kdl);
  sensor_to_desired_frame.header.frame_id = ft_frame_id_[0];
  sensor_to_desired_frame.header.stamp = ros::Time(0);
  sensor_to_desired_frame.pose.position.x = 0;
  sensor_to_desired_frame.pose.position.y = 0;
  sensor_to_desired_frame.pose.position.z = 0;
  sensor_to_desired_frame.pose.orientation.x = 0;
  sensor_to_desired_frame.pose.orientation.y = 0;
  sensor_to_desired_frame.pose.orientation.z = 0;
  sensor_to_desired_frame.pose.orientation.w = 1;
  desired_frame_to_base = sensor_to_desired_frame;
  desired_frame_to_base.header.frame_id = frame;

  try
  {
    // obtain a vector from the wrench frame id to the desired ft frame
    listener_.transformPose(frame, sensor_to_desired_frame, sensor_to_desired_frame);
    listener_.transformPose(base_link_, desired_frame_to_base, desired_frame_to_base);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
  }
  
  tf::poseMsgToKDL(sensor_to_desired_frame.pose, sensor_to_desired_frame_kdl);
  tf::poseMsgToKDL(desired_frame_to_base.pose, desired_frame_to_base_kdl);
  wrench_kdl = sensor_to_desired_frame_kdl*wrench_kdl;
  tf::wrenchKDLToEigen(wrench_kdl, converted_wrench);
  return converted_wrench;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::startActionlib()
{
  // Initialize actionlib server
  action_server_ = boost::shared_ptr<actionlib::SimpleActionServer<ActionClass> >(new actionlib::SimpleActionServer<ActionClass>(nh_, action_name_, false));

  // Register callbacks
  action_server_->registerGoalCallback(boost::bind(&ControllerTemplate::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&ControllerTemplate::preemptCB, this));

  action_server_->start();

  ROS_INFO("%s initialized successfully!", action_name_.c_str());
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::hasJoint(const KDL::Chain &chain, const std::string &joint_name)
{
  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    if(chain.segments[i].getJoint().getName() == joint_name)
    {
      return true;
    }
  }

  return false;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
int ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getJointIndex(const std::vector<std::string> &joint_names, const std::string &name)
{
  for (unsigned long i = 0; i < joint_names.size(); i++)
  {
    if (joint_names[i] == name)
    {
      return i;
    }
  }

  throw std::logic_error("getJointIndex: Tried to query a joint name that is not present in the joint names vector.");
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker)
{
  geometry_msgs::Point point;

  marker.points.clear();
  tf::pointEigenToMsg(initial_point, point);
  marker.points.push_back(point);
  tf::pointEigenToMsg(final_point, point);
  marker.points.push_back(point);
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::vectorMsgToStd(const geometry_msgs::Vector3 &in, std::vector<double> &out)
{
  if (out.size() != 3)
  {
    throw std::logic_error("vectorMsgToStd: out must be dimension 3");
  }

  out[0] = in.x;
  out[1] = in.y;
  out[2] = in.z;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::vectorStdToMsg(const std::vector<double> &in, geometry_msgs::Vector3 &out)
{
  if (in.size() != 3)
  {
    throw std::logic_error("vectorStdToMsg: in must be dimension 3");
  }

  out.x = in[0];
  out.y = in[1];
  out.z = in[2];
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getParam(const std::string &param_name, std::string &var)
{
  if (!nh_.getParam(param_name, var))
  {
    ROS_ERROR("Missing ROS parameter %s!", param_name.c_str());
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getParam(const std::string &param_name, double &var)
{
  if (!nh_.getParam(param_name, var))
  {
    ROS_ERROR("Missing ROS parameter %s!", param_name.c_str());
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getParam(const std::string &param_name, std::vector<double> &var)
{
  if (!nh_.getParam(param_name, var))
  {
    ROS_ERROR("Missing ROS parameter %s!", param_name.c_str());
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getParam(const std::string &param_name, int &var)
{
  if (!nh_.getParam(param_name, var))
  {
    ROS_ERROR("Missing ROS parameter %s!", param_name.c_str());
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getParam(const std::string &param_name, bool &var)
{
  if (!nh_.getParam(param_name, var))
  {
    ROS_ERROR("Missing ROS parameter %s!", param_name.c_str());
    return false;
  }

  return true;
}

template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::loadGenericParams()
{
  for (int i = 0; i < NUM_ARMS; i++)
  {
    if(!getParam("/common/end_effector_link_name/arm_" + std::to_string(i + 1), end_effector_link_[i]))
    {
      return false;
    }

    if (!getParam("/common/force_torque_frame/arm_" + std::to_string(i + 1), ft_frame_id_[i])) // this is the frame where we want to transform the force/torque data
    {
      return false;
    }

    if (!getParam("/common/force_torque_sensor_frame/arm_" + std::to_string(i + 1), ft_sensor_frame_[i]))
    {
      return false;
    }

    if (!getParam("/common/force_torque_topic/arm_" + std::to_string(i + 1), ft_topic_name_[i]))
    {
      return false;
    }
  }

  if (!getParam("/common/base_link_name", base_link_))
  {
    return false;
  }

  if (!getParam("/common/chain_base_link_name", chain_base_link_))
  {
    return false;
  }

  if (!getParam("/common/solver/epsilon", eps_))
  {
    return false;
  }

  if (!getParam("/common/solver/alpha", alpha_))
  {
    return false;
  }

  if (!getParam("/common/solver/maxiter", maxiter_))
  {
    return false;
  }

  if (!getParam("/common/solver/nso_weights", nso_weights_))
  {
    return false;
  }

  if (!getParam("/common/feedback_rate", feedback_hz_))
  {
    return false;
  }

  if(!model_.initParam("/robot_description")){
      ROS_ERROR("ERROR getting robot description (/robot_description)");
      return false;
  }

  return true;
}
}
#endif

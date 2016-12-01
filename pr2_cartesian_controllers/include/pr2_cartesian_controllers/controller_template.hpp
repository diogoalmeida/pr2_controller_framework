#ifndef __CONTROLLER_TEMPLATE__
#define __CONTROLLER_TEMPLATE__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <actionlib/server/simple_action_server.h>

namespace cartesian_controllers{

// Makes life easier for instantiating a pointer to a controller
class ControllerBase
{
public:
  ControllerBase(){}
  virtual ~ControllerBase(){}

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt) = 0;
};

template <class ActionClass, class ActionFeedback, class ActionResult>
class ControllerTemplate : public ControllerBase
{
public:
  ControllerTemplate();
  virtual ~ControllerTemplate()
  {
    delete fkpos_;
    delete ikpos_;
    delete ikvel_;

    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }
  }

protected:
  // Robot related
  sensor_msgs::JointState robot_state;
  KDL::JntArray joint_positions_;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::ChainIkSolverPos_NR_JL *ikpos_limits_;
  KDL::ChainIkSolverPos_LMA *ikpos_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_, base_link_, ft_topic_name_;
  double eps_; // weighted damped least squares epsilon
  double feedback_hz_;
  bool has_state_;
  sensor_msgs::JointState last_state_;
  sensor_msgs::JointState lastState(const sensor_msgs::JointState current);

  //Actionlib
  actionlib::SimpleActionServer<ActionClass> *action_server_;
  ActionFeedback feedback_;
  ActionResult result_;
  std::string action_name_;

  virtual void goalCB() = 0;
  virtual void preemptCB() = 0;
  void startActionlib();

  boost::thread feedback_thread_;
  boost::mutex reference_mutex_;

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber ft_sub_;
  tf::TransformListener listener_;

  bool loadGenericParams();
  virtual bool loadParams() = 0;
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  Eigen::Matrix<double, 6, 1> measured_wrench_;
};

// Implementing the template class in its header file saves some headaches
// later on: http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor

template <class ActionClass, class ActionFeedback, class ActionResult>
ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::ControllerTemplate()
{
  nh_ = ros::NodeHandle("~");

  if(!loadGenericParams())
  {
    ros::shutdown();
    exit(0);
  }

  // Initialize KDL variables
  KDL::JntArray q_min(7); // minimum limits of each joint
  KDL::JntArray q_max(7); // maximum limits of each joint

  joint_positions_.resize(7);
  kdl_parser::treeFromUrdfModel(model_, tree_); // convert URDF description of the robot into a KDL tree
  tree_.getChain(base_link_, end_effector_link_, chain_);

  KDL::Joint kdl_joint;
  boost::shared_ptr<const urdf::Joint> urdf_joint;
  int j = 0;
  for (int i = 0; i < chain_.getNrOfSegments(); i++) // get joint limits
  {
    kdl_joint = chain_.getSegment(i).getJoint();

    if (kdl_joint.getTypeName() == "None")
    {
      continue;
    }

    ROS_INFO("Getting joint limits for joint %s", kdl_joint.getName().c_str());
    urdf_joint = model_.getJoint(kdl_joint.getName());
    q_min(j) = urdf_joint->limits->lower;
    q_max(j) = urdf_joint->limits->upper;
    j++;
  }

  ROS_INFO("j: %d", j);

  fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);
  ikpos_limits_ = new KDL::ChainIkSolverPos_NR_JL(chain_, q_min, q_max, *fkpos_, *ikvel_, 100, 0.001);
  ikpos_ = new KDL::ChainIkSolverPos_LMA(chain_);
  has_state_ = false;

  // Subscribe to force and torque measurements
  measured_wrench_ << 0, 0, 0, 0, 0, 0;
  ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ControllerTemplate::forceTorqueCB, this);
}

/*
  Return last controlled joint state
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
sensor_msgs::JointState ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::lastState(const sensor_msgs::JointState current)
{
  if(!has_state_)
  {
    last_state_ = current;
    for (int i = 0; i < last_state_.velocity.size(); i++)
    {
      last_state_.velocity[i] = 0.0;
    }

    has_state_ = true;
  }

  return last_state_;
}

/*
  Update current force and torque values.
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  geometry_msgs::Vector3Stamped vector_in, vector_out;
  geometry_msgs::Wrench transformed_wrench;
  boost::lock_guard<boost::mutex> guard(reference_mutex_);

  vector_in.vector = msg->wrench.torque;
  vector_in.header = msg->header;
  vector_in.header.stamp = ros::Time(0); // To enable transform with the latest ft data available

  try
  {
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.torque = vector_in.vector;
    vector_in.vector = msg->wrench.force;
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.force = vector_in.vector;

    tf::wrenchMsgToEigen(transformed_wrench, measured_wrench_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
  }
}

/*
  Code for starting the actionlib server
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::startActionlib()
{
  // Initialize actionlib server
  action_server_ = new actionlib::SimpleActionServer<ActionClass>(nh_, action_name_, false);

  // Register callbacks
  action_server_->registerGoalCallback(boost::bind(&ControllerTemplate::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&ControllerTemplate::preemptCB, this));

  action_server_->start();

  ROS_INFO("%s initialized successfully!", action_name_.c_str());
}

/*
  Search for generic controller parameters
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::loadGenericParams()
{
  if (!nh_.getParam("/common/end_effector_link_name", end_effector_link_))
  {
    ROS_ERROR("Missing end-effector link name (/common/end_effector_link_name)");
    return false;
  }

  if (!nh_.getParam("/common/base_link_name", base_link_))
  {
    ROS_ERROR("Missing base link name (/common/base_link_name)");
    return false;
  }

  if (!nh_.getParam("/common/wdls_epsilon", eps_))
  {
    ROS_ERROR("Missing wdls epsilon (/common/wdls_epsilon)");
    return false;
  }

  if (!nh_.getParam("/common/feedback_rate", feedback_hz_))
  {
    ROS_ERROR("Missing feedback_rate (/common/feedback_rate)");
    return false;
  }

  if (!nh_.getParam("/common/force_torque_topic", ft_topic_name_))
  {
    ROS_ERROR("Missing force torque topic name (/common/force_torque_topic)");
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

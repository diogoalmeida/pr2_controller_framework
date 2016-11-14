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

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt) = 0;
};

template <class ActionClass, class ActionFeedback, class ActionResult>
class ControllerTemplate : public ControllerBase
{
public:
  ControllerTemplate();

protected:
  // Robot related
  sensor_msgs::JointState robot_state;
  KDL::JntArray joint_positions_;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::ChainIkSolverPos_LMA *ikpos_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_, base_link_, ft_topic_name_;
  double eps_; // weighted damped least squares epsilon
  double feedback_hz_;

  //Actionlib
  actionlib::SimpleActionServer<ActionClass> *action_server_;
  ActionFeedback feedback_;
  ActionResult result_;
  std::string action_name_;

  virtual void goalCB() = 0;
  virtual void preemptCB() = 0;
  void startActionlib();

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
  joint_positions_.resize(7);
  kdl_parser::treeFromUrdfModel(model_, tree_); // convert URDF description of the robot into a KDL tree
  tree_.getChain(base_link_, end_effector_link_, chain_);
  fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ikpos_ = new KDL::ChainIkSolverPos_LMA(chain_);
  ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);

  // Subscribe to force and torque measurements
  ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ControllerTemplate::forceTorqueCB, this);
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
  listener_.transformVector(base_link_, vector_in, vector_out);
  transformed_wrench.torque = vector_in.vector;

  vector_in.vector = msg->wrench.force;
  listener_.transformVector(base_link_, vector_in, vector_out);
  transformed_wrench.force = vector_in.vector;

  tf::wrenchMsgToEigen(transformed_wrench, measured_wrench_);
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

  if(!model_.initParam("/robot_description")){
      ROS_ERROR("ERROR getting robot description (/robot_description)");
      return false;
  }

  return true;
}
}
#endif

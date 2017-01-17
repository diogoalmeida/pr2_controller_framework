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
  // KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::ChainIkSolverVel_pinv_nso *ikvel_;
  KDL::ChainIkSolverPos_LMA *ikpos_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_, base_link_, ft_topic_name_, ft_frame_id_;
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
  ros::Publisher ft_pub_;
  tf::TransformListener listener_;

  bool loadGenericParams();
  virtual bool loadParams() = 0;
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void getJointLimits(KDL::JntArray &min_limits, KDL::JntArray &max_limits);
  Eigen::Matrix<double, 6, 1> measured_wrench_;
  double force_d_;

private:
  tf::TransformBroadcaster broadcaster_;
};

// Implementing the template class in its header file saves some headaches
// later on: http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor

template <class ActionClass, class ActionFeedback, class ActionResult>
ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::ControllerTemplate()
{
  KDL::JntArray min_limits, max_limits;
  KDL::JntArray optimal_values, weights;

  nh_ = ros::NodeHandle("~");

  if(!loadGenericParams())
  {
    ros::shutdown();
    exit(0);
  }

  kdl_parser::treeFromUrdfModel(model_, tree_); // convert URDF description of the robot into a KDL tree
  tree_.getChain(base_link_, end_effector_link_, chain_);
  getJointLimits(min_limits, max_limits);
  ROS_INFO("Min limits rows: %d, min limits columns: %d", min_limits.rows(), min_limits.columns());
  joint_positions_.resize(chain_.getNrOfJoints());
  optimal_values.resize(chain_.getNrOfJoints());
  weights.resize(chain_.getNrOfJoints());

  ROS_INFO("Joint limits: ");
  for (int i = 0; i < chain_.getNrOfJoints(); i++) // define the optimal joint values as the one that's as far away from joint limits as possible
  {
    optimal_values(i) = (min_limits(i) + max_limits(i))/2;

    ROS_INFO("Joint: %d, min_limit: %.2f, max_limit: %.2f, optimal_value: %.2f", i, min_limits(i), max_limits(i), optimal_values(i));

    if (min_limits(i) == max_limits(i)) // Do not weight in joints with no limits in the nullspace optimization method
    {
      weights(i) = 0;
    }
    else
    {
      weights(i) = 0.1;
    }

    ROS_INFO("Weight: %.2f\n\n", weights(i));
  }


  fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);
  // ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);
  // ikvel_ = new KDL::ChainIkSolverVel_pinv_nso(chain_, eps_);
  ikvel_ = new KDL::ChainIkSolverVel_pinv_nso(chain_, optimal_values, weights, eps_);
  ikpos_ = new KDL::ChainIkSolverPos_LMA(chain_);
  has_state_ = false;

  // Subscribe to force and torque measurements
  measured_wrench_ << 0, 0, 0, 0, 0, 0;
  ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ControllerTemplate::forceTorqueCB, this);
  ft_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ft_topic_name_ + "/converted", 1);
}

/*
  Use the controller kinematic chain to obtain the joint limits of the manipulator
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::getJointLimits(KDL::JntArray &min_limits, KDL::JntArray &max_limits)
{
  KDL::Joint kdl_joint;
  boost::shared_ptr<const urdf::Joint> urdf_joint;
  int j = 0;

  min_limits.resize(chain_.getNrOfJoints());
  max_limits.resize(chain_.getNrOfJoints());

  for (int i = 0; i < chain_.getNrOfSegments(); i++) // get joint limits
  {
    kdl_joint = chain_.getSegment(i).getJoint();

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
  Update current force and torque values, and transform them to the
  desired ft frame id.
*/
template <class ActionClass, class ActionFeedback, class ActionResult>
void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  KDL::Wrench wrench_kdl;
  KDL::Frame sensor_to_grasp_frame_kdl, sensor_frame_kdl, desired_kdl;
  geometry_msgs::PoseStamped sensor_to_grasp_frame, sensor_frame, desired;
  geometry_msgs::WrenchStamped converted_wrench;
  tf::Transform converted_wrench_frame;

  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  tf::wrenchMsgToKDL(msg->wrench, wrench_kdl);

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
  converted_wrench.header.frame_id = ft_frame_id_;

  try
  {
    // obtain a vector from the wrench frame id to the desired ft frame
    listener_.transformPose(ft_frame_id_, sensor_to_grasp_frame, sensor_to_grasp_frame);
    // listener_.transformPose(base_link_, sensor_frame, sensor_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF exception in %s: %s", action_name_.c_str(), ex.what());
  }

  tf::poseMsgToKDL(sensor_to_grasp_frame.pose, sensor_to_grasp_frame_kdl);
  wrench_kdl = sensor_to_grasp_frame_kdl*wrench_kdl;
  tf::wrenchKDLToMsg(wrench_kdl, converted_wrench.wrench);
  tf::wrenchKDLToEigen(wrench_kdl, measured_wrench_);
  ft_pub_.publish(converted_wrench);
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

  if (!nh_.getParam("/common/force_torque_frame", ft_frame_id_)) // this is the frame where we want to transform the force/torque data
  {
    ROS_ERROR("Missing force torque frame name (/common/force_torque_frame)");
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

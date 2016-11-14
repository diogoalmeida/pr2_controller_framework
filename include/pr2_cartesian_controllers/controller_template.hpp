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

template <class ActionClass, class ActionFeedback, class ActionResult>
class ControllerTemplate
{
public:
  ControllerTemplate();

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt) = 0;

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

  virtual void publishFeedback();
  virtual void goalCB() = 0;
  virtual void preemptCB() = 0;

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
}
#endif

#ifndef __APPROACH_CONTROLLER__
#define __APPROACH_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>

namespace manipulation{

class ApproachController : public cartesian_controllers::ControllerTemplate
{
private:
  // Robot related
  sensor_msgs::JointState robot_state;
  KDL::JntArray joint_positions_;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_, base_link_, ft_topic_name_;
  double eps_; // weighted damped least squares epsilon

  boost::mutex reference_mutex_;

  // Actionlib
  actionlib::SimpleActionServer<pr2_cartesian_controllers::GuardedApproachAction> *action_server_;
  pr2_cartesian_controllers::GuardedApproachFeedback feedback_;
  pr2_cartesian_controllers::GuardedApproachResult result_;
  std::string action_name_;
  void publishFeedback();
  void goalCB();
  void preemptCB();

  double feedback_hz_;

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber ft_sub_;
  tf::TransformListener listener_;

  bool loadParams();
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  // Controller values
  Eigen::Matrix<double, 6, 1> measured_wrench_;

public:
  ApproachController();

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

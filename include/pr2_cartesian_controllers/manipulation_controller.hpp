#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
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

class ManipulationController : public cartesian_controllers::ControllerTemplate
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
  actionlib::SimpleActionServer<pr2_cartesian_controllers::ManipulationControllerAction> *action_server_;
  pr2_cartesian_controllers::ManipulationControllerFeedback feedback_;
  pr2_cartesian_controllers::ManipulationControllerResult result_;
  std::string action_name_;
  void publishFeedback();
  void goalCB();
  void preemptCB();

  // TODO: convert the PR2 controller class into a joint controller that gets inputs
  // from one of several possible and mutually exclusive action servers. Several different
  // joint controllers, each loading their own al server?
  // actionlib::SimpleActionServer<manipulation_controller::GuardedApproachAction> *approach_action_server_;
  // manipulation_controller::GuardedApproachFeedback approach_feedback_;
  // manipulation_controller::GuardedApproachResult approach_result_;
  // std::string approach_action_name_;
  // void approachPublishFeedback();
  // void approachGoalCB();
  // void approachPreemptCB();

  double feedback_hz_;

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber ft_sub_;
  tf::TransformListener listener_;

  bool loadParams();
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  // Controller values
  double k_spring_, estimated_length_, estimated_orientation_;
  Eigen::Affine3d surface_frame_, goal_pose_, end_effector_pose_;
  Eigen::Matrix<double, 6, 1> measured_wrench_;
  Eigen::Matrix3d control_gains_;
  Eigen::Vector3d estimated_r_;
  void estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt);

public:
  ManipulationController();

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

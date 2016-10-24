#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <manipulation_controller/ManipulationControllerAction.h>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <actionlib/server/simple_action_server.h>

namespace manipulation{

class ManipulationController
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
  actionlib::SimpleActionServer<manipulation_controller::ManipulationControllerAction> *action_server_;
  manipulation_controller::ManipulationControllerFeedback feedback_;
  manipulation_controller::ManipulationControllerResult result_;
  std::string action_name_;
  double feedback_hz_;
  void publishFeedback();
  void goalCB();
  void preemptCB();

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
  Eigen::Vector3d estimated_r_;

public:
  ManipulationController();

  // Control topic: meant to be called in the realtime loop
  sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

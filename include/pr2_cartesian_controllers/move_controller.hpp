#ifndef __APPROACH_CONTROLLER__
#define __APPROACH_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>

namespace manipulation{

class MoveController : public cartesian_controllers::ControllerTemplate
{
private:
  // Robot related
  sensor_msgs::JointState robot_state;
  KDL::JntArray joint_positions_;
  KDL::ChainIkSolverPos_LMA *ikpos_;
  KDL::ChainFkSolverPos_recursive *fkpos_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_, base_link_;

  boost::mutex reference_mutex_;

  // Actionlib
  actionlib::SimpleActionServer<pr2_cartesian_controllers::MoveAction> *action_server_;
  pr2_cartesian_controllers::MoveFeedback feedback_;
  pr2_cartesian_controllers::MoveResult result_;
  std::string action_name_;
  void goalCB();
  void preemptCB();

  // ROS
  ros::NodeHandle nh_;
  tf::TransformListener listener_;

  bool loadParams();

  // Controller values
  KDL::Frame pose_reference_;

public:
  MoveController();

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

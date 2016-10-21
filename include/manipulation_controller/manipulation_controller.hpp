#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <sensor_msgs/JointState.h>
#include <manipulation_controller/ManipulationControllerAction.h>
#include <boost/thread.hpp>
#include <urdf/model.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <actionlib/server/simple_action_server.h>

namespace manipulation{

class ManipulationController
{
private:
  // Robot related
  sensor_msgs::JointState robot_state;
  KDL::ChainIkSolverVel_wdls *ikvel_;
  KDL::Chain chain_;
  KDL::Tree tree_;
  urdf::Model model_;
  std::string end_effector_link_;

  // Actionlib
  actionlib::SimpleActionServer<manipulation_controller::ManipulationControllerAction> *action_server_;
  manipulation_controller::ManipulationControllerFeedback feedback_;
  manipulation_controller::ManipulationControllerResult result_;
  std::string action_name_;
  double feedback_hz_;
  void publishFeedback();
  void goalCB();
  void preemptCB();
  bool loadParams();

  // ROS
  ros::NodeHandle nh_;

public:
  ManipulationController();

  // Control topic: meant to be called in the realtime loop
  sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

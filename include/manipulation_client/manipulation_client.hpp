#ifndef __MANIPULATION_CLIENT__
#define __MANIPULATION_CLIENT__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/MoveAction.h>
#include <std_srvs/Empty.h>

namespace manipulation{

  class ManipulationClient
  {
  public:
    ManipulationClient()
    {
      nh_ = ros::NodeHandle("~");
      runExperiment();
    }

  private:
    // ros
    ros::NodeHandle nh_;
    ros::ServiceClient gravity_compensation_client_;
    tf::TransformListener listener_;
    bool loadParams();

    // actionlib
    actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction> *manipulation_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction> *approach_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction> *move_action_client_;
    std::string move_action_name_, manipulation_action_name_, approach_action_name_;
    double server_timeout_;

    // Vision feedback
    double vision_timeout_;
    bool waitForTablePose(ros::Duration max_time);
    std::string surface_frame_name_;

    // Experimental setup
    void runExperiment();
    std::vector<double> initial_pose_offset_;
    double initial_approach_angle_;
    geometry_msgs::PoseStamped surface_frame_pose_;
    geometry_msgs::PoseStamped initial_eef_pose_;
    std::string base_link_name_, tool_frame_name_;
    std::string gravity_compensation_service_name_;
    int num_of_experiments_;
  };
  }
#endif

#ifndef __MANIPULATION_CLIENT__
#define __MANIPULATION_CLIENT__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>

namespace manipulation{

  class ManipulationClient
  {
  public:
    ManipulationClient(std::string action_name) : action_client_(action_name, true)
    {
      nh_ = ros::NodeHandle("~");
      action_name_ = action_name;
      runExperiment();
    }

  private:
    // ros
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    bool loadParams();

    // actionlib
    actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction> action_client_;
    std::string action_name_;
    double server_timeout_;

    // Vision feedback
    double vision_timeout_;
    bool waitForTablePose(ros::Duration max_time);
    std::string surface_frame_name_;

    // Experimental setup
    void runExperiment();
    std::vector<double> initial_pose_offset_;
    geometry_msgs::PoseStamped surface_frame_pose_;
    geometry_msgs::PoseStamped initial_eef_pose_;
    std::string base_link_name_, tool_frame_name_;
    int num_of_experiments_;
  };
  }
#endif

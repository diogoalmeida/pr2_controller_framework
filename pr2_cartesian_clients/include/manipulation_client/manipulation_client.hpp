#ifndef __MANIPULATION_CLIENT__
#define __MANIPULATION_CLIENT__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_clients/ManipulationAction.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>

namespace manipulation{

  class ManipulationClient
  {
  public:
    ManipulationClient();
    ~ManipulationClient();
    void runExperiment();

  private:
    // boost
    boost::thread feedback_thread_;
    boost::mutex reference_mutex_;

    // ros
    ros::NodeHandle nh_;
    ros::ServiceClient gravity_compensation_client_;
    ros::ServiceClient load_controllers_client_;
    ros::ServiceClient unload_controllers_client_;
    ros::ServiceClient switch_controllers_client_;
    tf::TransformListener listener_;
    std::string move_controller_name_, manipulation_controller_name_, approach_controller_name_;
    bool loadParams();
    bool loadControllers();
    bool loadController(std::string controller_name);
    bool unloadControllers();
    bool unloadController(std::string controller_name);
    bool switchToController(std::string controller_name);

    // actionlib
    actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction> *manipulation_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction> *approach_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction> *move_action_client_;
    actionlib::SimpleActionServer<pr2_cartesian_clients::ManipulationAction> *action_server_; // Allows user-triggered preemption
    std::string move_action_name_, manipulation_action_name_, approach_action_name_, cartesian_client_action_name_, current_action_;
    double server_timeout_, feedback_hz_;
    pr2_cartesian_clients::ManipulationFeedback feedback_;
    void publishFeedback();
    void goalCB();
    void preemptCB();

    // Vision feedback
    double vision_timeout_;
    bool waitForTablePose(ros::Duration max_time);
    std::string surface_frame_name_;
    bool getInitialEefPose(geometry_msgs::PoseStamped & pose);

    // Experimental setup
    std::vector<double> initial_pose_offset_;
    double initial_approach_angle_;
    geometry_msgs::PoseStamped surface_frame_pose_;
    std::string base_link_name_, tool_frame_name_;
    std::string gravity_compensation_service_name_;
    int num_of_experiments_;
    bool use_vision_, sim_mode_;

    // others
    void destroyActionClients();
  };
  }
#endif

#ifndef __MANIPULATION_CLIENT__
#define __MANIPULATION_CLIENT__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/GuardedApproachAction.h>
#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_clients/ManipulationAction.h>
#include <pr2_cartesian_clients/LogMessages.h>
#include <utils/extra.hpp>
#include <utils/ExclusiveControllerRunner.hpp>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>

using namespace pr2_cartesian_clients;
namespace manipulation{

  class ManipulationClient
  {
  public:
    ManipulationClient();
    ~ManipulationClient();

    /**
      Initializes the robot pose based on tag information. Commands the initial
      approach. Monitors vision information to compute
      ground truth data, and registers feedback.
    **/
    void runExperiment();

  private:
    /**
      When receiving a new goal, initialize the action clients
      and get vision information
    **/
    void goalCB();

    /**
      Upon preemption, cancel all goals and destroy the action clients
    **/
    void preemptCB();

    /**
      Get the client parameters and abort in case they are not found.
    **/
    bool loadParams();

    /**
      Periodically publish feedback reporting the action being run.
    **/
    void publishFeedback();

    /*
      Waits for the table frame to be available and saves the frame
      data.

      @param max_time The maximum wait time.
      @return True if a table pose was found.
    */
    bool waitForTablePose(ros::Duration max_time);

    /**
      Gets the initial eef pose, based on vision or a pre-set value.

      @param pose The desired initial eef pose.
      @return false if it takes more than vision_timeout_ seconds to obtain a valid
      surface frame transform
    **/
    bool getInitialEefPose(geometry_msgs::PoseStamped & pose);

    /**
      Makes sure the action clients cancel the goals and are properly deleted.
    **/
    void destroyActionClients();

  private:
    boost::thread feedback_thread_;
    boost::mutex reference_mutex_;

    ros::NodeHandle nh_;
    ros::ServiceClient gravity_compensation_client_, logging_service_client_;
    tf::TransformListener listener_;
    std::string move_controller_name_, manipulation_controller_name_, approach_controller_name_, bag_prefix_;

    actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction> *manipulation_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction> *approach_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction> *move_action_client_;
    actionlib::SimpleActionServer<pr2_cartesian_clients::ManipulationAction> *action_server_; // Allows user-triggered preemption
    std::string move_action_name_, manipulation_action_name_, approach_action_name_, cartesian_client_action_name_, current_action_, logging_service_;
    double server_timeout_, feedback_hz_;
    double move_action_time_limit_, approach_action_time_limit_, manipulation_action_time_limit_;
    pr2_cartesian_clients::ManipulationFeedback feedback_;

    double vision_timeout_;
    std::string surface_frame_name_;

    std::vector<double> initial_pose_offset_;
    double initial_approach_angle_, approach_velocity_, approach_force_;
    double goal_x_, goal_theta_, goal_force_;
    geometry_msgs::PoseStamped surface_frame_pose_;
    std::string base_link_name_, tool_frame_name_;
    std::string gravity_compensation_service_name_;
    int num_of_experiments_, current_iter_, arm_;
    bool use_vision_, sim_mode_;
    pr2_cartesian_clients::ExclusiveControllerRunner controller_runner_;

    std::default_random_engine noise_generator_;
    std::uniform_real_distribution<double> noise_x_d_;
    std::uniform_real_distribution<double> noise_theta_d_;
    std::uniform_real_distribution<double> noise_f_d_;
  };
  }
#endif

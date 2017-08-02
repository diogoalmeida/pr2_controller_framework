#ifndef __MECHANISM_CLIENT__
#define __MECHANISM_CLIENT__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pr2_cartesian_controllers/MechanismIdentificationAction.h>
#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_clients/MechanismAction.h>
#include <pr2_cartesian_clients/LogMessages.h>
#include <utils/extra.hpp>
#include <utils/ExclusiveControllerRunner.hpp>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>

using namespace pr2_cartesian_clients;
namespace manipulation{

  class MechanismClient
  {
  public:
    MechanismClient();
    ~MechanismClient();

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
      Get a pose from the parameter serve. Assumed to be in the 'torso_lift_link' frame.

      @param param The parameter server directory.
      @param pose The pose.
    **/
    bool getPose(const std::string &param, geometry_msgs::PoseStamped &pose);

    /**
      Periodically publish feedback reporting the action being run.
    **/
    void publishFeedback();

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
    std::string move_controller_name_, mechanism_controller_name_, bag_prefix_;

    actionlib::SimpleActionClient<pr2_cartesian_controllers::MechanismIdentificationAction> *mechanism_action_client_;
    actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction> *move_action_client_;
    actionlib::SimpleActionServer<pr2_cartesian_clients::MechanismAction> *action_server_; // Allows user-triggered preemption
    std::string move_action_name_, mechanism_action_name_, approach_action_name_, cartesian_client_action_name_, current_action_, logging_service_;
    double server_timeout_, feedback_hz_;
    double move_action_time_limit_, mechanism_action_time_limit_;
    pr2_cartesian_clients::MechanismFeedback feedback_;

    double vd_amp_, vd_freq_, wd_amp_, wd_freq_, goal_force_, km_;
    geometry_msgs::PoseStamped initial_rod_pose_, initial_surface_pose_;
    std::string base_link_name_, surface_frame_name_;
    std::string gravity_compensation_service_name_;
    int num_of_experiments_, current_iter_, rod_arm_, surface_arm_;
    bool use_vision_, sim_mode_, use_nullspace_, use_estimates_;
    pr2_cartesian_clients::ExclusiveControllerRunner controller_runner_;

    std::default_random_engine noise_generator_;
    std::uniform_real_distribution<double> noise_x_d_;
    std::uniform_real_distribution<double> noise_theta_d_;
    std::uniform_real_distribution<double> noise_f_d_;
  };
  }
#endif

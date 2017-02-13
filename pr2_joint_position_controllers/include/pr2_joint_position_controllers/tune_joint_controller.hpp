#ifndef __TUNE_JOINT_CONTROLLER__
#define __TUNE_JOINT_CONTROLLER__

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>
#include <pr2_joint_position_controllers/PR2TuneJointAction.h>

namespace pr2_joint_controller{

/**
  Implements a controller that allows for tuning the nested PID control loop
  that is provided by the template joint controller.

  Exposes an actionlib interface that gives the user the ability to send a reference
  to the joint controller and to adjust the controller gains.
  Published the required feedback to monitor the controller response.
**/
class TuneJointController: public pr2_controller_interface::Controller
{
private:
  /**
    Apply position feedback, add it to the velocity reference (which acts as a
    feedforward term) and then apply the velocity feedback.

    @param joint_state The current state of the controlled joint.
    @param desired_position The reference position for the joint.
    @param desired_velocity The reference velocity for the joint.
    @param dt The elapsed time since the last call.

    @return The commanded effort to the joint.
  **/
  double applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, ros::Duration dt); // applies the nested control strategy

  /**
    The actionlib goal callback thread. Accepts a new goal consisting of the
    joint we want to tune and the desired PID parameters.
  **/
  void goalCallback();

  /**
    The actionlib preempt callback thread.
  **/
  void preemptCallback();

  /**
    The feedback thread, running at a non-realtime rate.
  **/
  void publishFeedback();

private:
  // PR2 state class
  pr2_mechanism_model::RobotState *robot_;

  // Control elements
  ros::NodeHandle n_;
  sensor_msgs::JointState control_reference_;
  control_toolbox::Pid* position_joint_controller_;
  control_toolbox::Pid* velocity_joint_controller_;
  std::string joint_name_;
  ros::Time start_time_, time_of_last_cycle_;
  ros::Duration max_time_;
  double modified_velocity_reference_;
  double ff_;

  // Reference mutex for preventing updating the reference halfway through a control cycle
  boost::mutex reference_mutex_;

  // Actionlib
  actionlib::SimpleActionServer<pr2_joint_position_controllers::PR2TuneJointAction> *action_server_;
  std::string action_name_;

  // Feedback elements
  ros::Publisher feedback_pub_;
  pr2_joint_position_controllers::PR2TuneJointFeedback feedback_;
  pr2_joint_position_controllers::PR2TuneJointResult result_;
  boost::thread feedback_thread_;
  double feedback_hz_;

public:
  ~TuneJointController()
  {
    if(feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }
  }
  // Controller interface methods
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
}

#endif

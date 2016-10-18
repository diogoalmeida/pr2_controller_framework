#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controller/PR2JointCommandAction.h>
#include <pr2_controller/PR2JointControlReference.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>

namespace pr2_joint_controller{

class JointController: public pr2_controller_interface::Controller
{
private:
  // PR2 state class
  pr2_mechanism_model::RobotState *robot_;

  // Control members
  sensor_msgs::JointState control_references_;
  std::vector<control_toolbox::Pid*> position_joint_controllers_;
  std::vector<control_toolbox::Pid*> velocity_joint_controllers_;
  std::vector<std::string> joint_names_; // the controller will only accept a command that includes ALL the joint_names_ it's expecting to receive. It ignores all other names
  std::vector<ros::Time> time_of_last_cycle_; // Per controller

  // Reference mutex for preventing updating the reference halfway through a control cycle
  boost::mutex reference_mutex_;

  // Actionlib interface members. Action server implemented with the Goal callback method.
  actionlib::SimpleActionServer<pr2_controller::PR2JointCommandAction> *action_server_;
  pr2_controller::PR2JointCommandFeedback feedback_;
  pr2_controller::PR2JointCommandResult result_;
  std::string action_name_;
  double feedback_hz_;

  // Listen to topic to update reference
  ros::Subscriber reference_subscriber_;
  ros::Duration control_timeout_; // Abort execution if no reference is received for longer than this
  ros::Time time_of_last_reference_update_;
  void referenceCallback(const pr2_controller::PR2JointControlReference::ConstPtr &msg);

  bool isActuatedJoint(std::string joint_name);
  double getReferencePosition(std::string joint_name);
  double getReferenceVelocity(std::string joint_name);
  void publishFeedback();

public:
  // Controller interface methods
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  // Actionlib interface methods
  void goalCB();
  void preemptCB();
};
}

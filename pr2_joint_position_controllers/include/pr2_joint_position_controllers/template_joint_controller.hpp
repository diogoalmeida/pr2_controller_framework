#ifndef __TEMPLATE_JOINT_CONTROLLER__
#define __TEMPLATE_JOINT_CONTROLLER__

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>
#include <pr2_joint_position_controllers/PR2JointControllerFeedback.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <pr2_cartesian_controllers/approach_controller.hpp>
#include <pr2_cartesian_controllers/manipulation_controller.hpp>
#include <pr2_cartesian_controllers/move_controller.hpp>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

namespace pr2_joint_controller{

class TemplateJointController: public pr2_controller_interface::Controller
{
private:
  // PR2 state class
  pr2_mechanism_model::RobotState *robot_;

  // Control elements
  sensor_msgs::JointState control_references_;
  std::vector<control_toolbox::Pid*> position_joint_controllers_;
  std::vector<control_toolbox::Pid*> velocity_joint_controllers_;
  std::vector<std::string> joint_names_; // the controller will only accept a command that includes ALL the joint_names_ it's expecting to receive. It ignores all other names
  std::vector<ros::Time> time_of_last_cycle_; // Per controller
  ros::Time time_of_last_manipulation_call_;
  std::vector<double> last_active_joint_position_; // Keeps the last recorded position of actuated joints while the controller is active
  std::vector<double> modified_velocity_references_; // Allows sending feedback of the true velocity control loop performance
  double applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, int controller_num, ros::Duration dt); // applies the nested control strategy

  // Reference mutex for preventing updating the reference halfway through a control cycle
  boost::mutex reference_mutex_;

  // Feedback elements
  ros::Publisher feedback_pub_;
  pr2_joint_position_controllers::PR2JointControllerFeedback feedback_;
  boost::thread feedback_thread_;
  double feedback_hz_;

  // Listen to topic to update reference
  ros::Subscriber reference_subscriber_;
  ros::Duration control_timeout_; // Abort execution if no reference is received for longer than this
  ros::Time time_of_last_reference_update_;
  void referenceCallback(const sensor_msgs::JointState::ConstPtr &msg);

  bool isActuatedJoint(std::string joint_name);
  double getReferencePosition(std::string joint_name);
  double getReferenceVelocity(std::string joint_name);
  void publishFeedback();
  void resetAllocableVariables();
  bool verifySanity(sensor_msgs::JointState &state);

  // Cartesian controller
  cartesian_controllers::ControllerBase *cartesian_controller_;

public:
  // Controller interface methods
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  // implementable method
  virtual cartesian_controllers::ControllerBase *initializeController() = 0;
};
}

#endif

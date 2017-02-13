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

/**
  Implements a nested position and velocity joint controller that is integrated
  with the PR2 realtime loop.

  It keeps a pointer to a cartesian controller, responsible for generating
  the joint references.
**/
class TemplateJointController: public pr2_controller_interface::Controller
{
private:
  /**
    Apply position feedback, add it to the velocity reference (which acts as a
    feedforward term) and then apply the velocity feedback.

    @param joint_state The current state of the controlled joint.
    @param desired_position The reference position for the joint.
    @param desired_velocity The reference velocity for the joint.
    @param controller_num The controller index in the controller vector.
    @param dt The elapsed time since the last call.

    @return The commanded effort to the joint.
  **/
  double applyControlLoop(const pr2_mechanism_model::JointState *joint_state, double desired_position, double desired_velocity, int controller_num, ros::Duration dt); // applies the nested control strategy
  void referenceCallback(const sensor_msgs::JointState::ConstPtr &msg);

  /**
    Check if the given joint name is actuated by the controller.

    @param joint_name The joint name.
    @return True, if the joint is meant to be controlled, false otherwise.
  **/
  bool isActuatedJoint(std::string joint_name);

  /**
    Get the position value in the control references for the given joint.
    It will search for the requested value in the control references.

    @param joint_name Desired joint name.
    @return The reference position for the given joint.
  **/
  double getReferencePosition(std::string joint_name);

  /**
    Get the velocity value in the control references for the given joint.
    It will search for the requested value in the control references.

    @param joint_name Desired joint name.
    @return The reference velocity for the given joint.
  **/
  double getReferenceVelocity(std::string joint_name);

  /**
    Thread responsible for publishing feedback information at a
    non-realtime rate.
  **/
  void publishFeedback();

  /**
    Frees all the allocated memory and clears all the std vectors that are initialized
    during the init() routine.
  **/
  void resetAllocableVariables();

  /**
    Allocates memory and initializes relevant vectors.

    @return True if successful. Will return false if required parameters are not found,
    or if joints are found which are not in the robot description.
  **/
  bool allocateVariables();

  /**
    Verifies if the given joint state obeys all the joint limits, and modifies it
    if not.

    @param state The system joint state.
    @return True if the joint state is within joint limits, false otherwise.
  **/
  bool verifySanity(sensor_msgs::JointState &state);

private:
  // PR2 state class
  pr2_mechanism_model::RobotState *robot_;

  ros::NodeHandle n_;

  // Control elements
  sensor_msgs::JointState control_references_;
  std::vector<control_toolbox::Pid*> position_joint_controllers_;
  std::vector<control_toolbox::Pid*> velocity_joint_controllers_;
  std::vector<std::string> joint_names_; // the controller will only accept a command that includes ALL the joint_names_ it's expecting to receive. It ignores all other names
  std::vector<ros::Time> time_of_last_cycle_; // Per controller
  ros::Time time_of_last_manipulation_call_;
  std::vector<double> last_active_joint_position_; // Keeps the last recorded position of actuated joints while the controller is active
  std::vector<double> modified_velocity_references_; // Allows sending feedback of the true velocity control loop performance
  std::vector<double> ff_joint_controllers_; // feedforward gain for the joint controllers

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

  // Cartesian controller
  cartesian_controllers::ControllerBase *cartesian_controller_;

public:
  /**
    Controller initialization method from the PR2 realtime loop.

    @param robot Pointer to the robot state in the PR2 mechanism.
    @return False in case of errors.
  **/
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);

  /**
    Controller startup in realtime. Allocates the required variables
    and setups the joint controllers.
  **/
  virtual void starting();

  /**
    Controller update loop in realtime. Called at 1kHz, must not
    block for more than 1ms.
  **/
  virtual void update();

  /**
    Routine for stopping the controller. Joins the feedback thread and
    resets the allocated variables.
  **/
  virtual void stopping();

  /**
    Initializes the cartesian controller that provides references to the
    joint position controllers.

    @return A pointer to the initialized controller.
  **/
  virtual cartesian_controllers::ControllerBase *initializeController() = 0;
};
}

#endif

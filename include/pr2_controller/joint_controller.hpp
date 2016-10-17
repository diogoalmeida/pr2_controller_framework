#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controller/PR2JointCommandAction.h>
#include <boost/thread/mutex.hpp>

namespace pr2_joint_controller{

class JointController: public pr2_controller_interface::Controller
{
private:
  double init_pos_;

  pr2_mechanism_model::JointState* joint_state_;
  pr2_mechanism_model::RobotState *robot_;

  control_toolbox::Pid pid_controller_;
  sensor_msgs::JointState control_references_;
  boost::mutex reference_mutex_;
  ros::Time time_of_last_cycle_;

  actionlib::SimpleActionServer<pr2_controller::PR2JointCommandAction> *action_server_;
  boost::shared_ptr<const pr2_controller::PR2JointCommandGoal> goal_;
  pr2_controller::PR2JointCommandFeedback feedback_;
  pr2_controller::PR2JointCommandResult result_;
  std::string action_name_;


public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  void goalCB();
  void preemptCB();
};
}

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>

namespace pr2_joint_controller{

class JointController: public pr2_controller_interface::Controller
{
private:
  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;
  control_toolbox::Pid pid_controller_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time time_of_last_cycle_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
}

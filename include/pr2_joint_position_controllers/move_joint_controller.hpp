#ifndef __MOVE_JOINT_CONTROLLER__
#define __MOVE_JOINT_CONTROLLER__

#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pr2_cartesian_controllers/move_controller.hpp>

namespace pr2_joint_controller{

class MoveJointController: public TemplateJointController
{
public:
  MoveJointController(){}
  virtual cartesian_controllers::ControllerTemplate *initializeController();
};
}

#endif

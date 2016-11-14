#ifndef __APPROACH_JOINT_CONTROLLER__
#define __APPROACH_JOINT_CONTROLLER__

#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pr2_cartesian_controllers/approach_controller.hpp>

namespace pr2_joint_controller{

class ApproachJointController: public TemplateJointController
{
public:
  ApproachJointController(){}
  virtual cartesian_controllers::ControllerBase *initializeController();
};
}

#endif

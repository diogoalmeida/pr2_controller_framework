#ifndef __MANIPULATION_JOINT_CONTROLLER__
#define __MANIPULATION_JOINT_CONTROLLER__

#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pr2_cartesian_controllers/manipulation_controller.hpp>

namespace pr2_joint_controller{

class ManipulationJointController: public TemplateJointController
{
public:
  ManipulationJointController();
  virtual cartesian_controllers::ControllerTemplate *initializeController();
};
}

#endif

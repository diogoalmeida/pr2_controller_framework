#ifndef __MECHANISM_JOINT_CONTROLLER__
#define __MECHANISM_JOINT_CONTROLLER__

#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pr2_cartesian_controllers/mechanism_identification_controller.hpp>

namespace pr2_joint_controller{

class MechanismJointController: public TemplateJointController
{
public:
  MechanismJointController(){}
  virtual boost::shared_ptr<cartesian_controllers::ControllerBase > initializeController();
};
}

#endif

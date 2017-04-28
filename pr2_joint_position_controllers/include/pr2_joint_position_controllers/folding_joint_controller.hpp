#ifndef __FOLDING_JOINT_CONTROLLER__
#define __FOLDING_JOINT_CONTROLLER__

#include <pr2_joint_position_controllers/template_joint_controller.hpp>
#include <pr2_cartesian_controllers/folding_controller.hpp>

namespace pr2_joint_controller{

class FoldingJointController: public TemplateJointController
{
public:
  FoldingJointController(){}
  virtual boost::shared_ptr<cartesian_controllers::ControllerBase > initializeController();
};
}

#endif

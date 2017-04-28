#include <pr2_joint_position_controllers/folding_joint_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

boost::shared_ptr<cartesian_controllers::ControllerBase > FoldingJointController::initializeController()
{
  return boost::shared_ptr<cartesian_controllers::ControllerBase >(new cartesian_controllers::FoldingController());
}
}

PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::FoldingJointController, pr2_controller_interface::Controller)

#include <pr2_joint_position_controllers/manipulation_joint_controller.hpp>
#include <pr2_cartesian_controllers/manipulation_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

cartesian_controllers::ControllerBase * ManipulationJointController::initializeController()
{
  return new cartesian_controllers::ManipulationController();
}
}

PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::ManipulationJointController, pr2_controller_interface::Controller)

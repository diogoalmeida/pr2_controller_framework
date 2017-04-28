#include <pr2_joint_position_controllers/move_joint_controller.hpp>
#include <pr2_cartesian_controllers/move_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

boost::shared_ptr<cartesian_controllers::ControllerBase > MoveJointController::initializeController()
{
  return boost::shared_ptr<cartesian_controllers::ControllerBase >(new cartesian_controllers::MoveController());
}
}

PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::MoveJointController, pr2_controller_interface::Controller)

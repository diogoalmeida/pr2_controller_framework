#include <pr2_joint_position_controllers/move_joint_controller.hpp>
#include <pr2_cartesian_controllers/move_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

/*
  Controller initialization
*/
cartesian_controllers::ControllerTemplate * MoveJointController::initializeController()
{
  return new manipulation::MoveController();
}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(pr2_joint_position_controllers, MoveJointController,
                         pr2_joint_controller::MoveJointController,
                         pr2_controller_interface::Controller)

#include <pr2_joint_position_controllers/approach_joint_controller.hpp>
#include <pr2_cartesian_controllers/approach_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

/*
  Controller initialization
*/
cartesian_controllers::ControllerTemplate * ApproachJointController::initializeController()
{
  return new manipulation::ApproachController();
}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::ApproachJointController, pr2_controller_interface::Controller)

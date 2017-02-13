#include <pr2_joint_position_controllers/approach_joint_controller.hpp>
#include <pr2_cartesian_controllers/approach_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

cartesian_controllers::ControllerBase * ApproachJointController::initializeController()
{
  return new cartesian_controllers::ApproachController();
}
}

PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::ApproachJointController, pr2_controller_interface::Controller)

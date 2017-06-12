#include <pr2_joint_position_controllers/mechanism_joint_controller.hpp>
#include <pr2_cartesian_controllers/mechanism_identification_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace pr2_joint_controller {

boost::shared_ptr<cartesian_controllers::ControllerBase > MechanismJointController::initializeController()
{
  return boost::shared_ptr<cartesian_controllers::ControllerBase >(new cartesian_controllers::MechanismIdentificationController());
}
}

PLUGINLIB_EXPORT_CLASS(pr2_joint_controller::MechanismJointController, pr2_controller_interface::Controller)

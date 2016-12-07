#ifndef __MANIPULATION_CONTROLLER__
#define __MANIPULATION_CONTROLLER__

#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <visualization_msgs/Marker.h>

namespace cartesian_controllers{

class ManipulationController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                                                pr2_cartesian_controllers::ManipulationControllerResult>
{
private:
  // Actionlib
  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

  // Controller values
  double k_spring_, estimated_length_, estimated_orientation_;
  Eigen::Affine3d surface_frame_, goal_pose_, end_effector_pose_;
  Eigen::Matrix3d control_gains_;
  Eigen::Vector3d estimated_r_;
  std::vector<double> rot_gains_;
  KDL::Frame initial_pose_;
  bool has_initial_;
  void estimatePose(const Eigen::Vector3d &rotation_axis, const Eigen::Vector3d &surface_tangent, const Eigen::Vector3d &surface_normal, ros::Duration dt);

public:
  ManipulationController() : ControllerTemplate<pr2_cartesian_controllers::ManipulationControllerAction,
                                                pr2_cartesian_controllers::ManipulationControllerFeedback,
                                                pr2_cartesian_controllers::ManipulationControllerResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    has_initial_ = false; // used to set the initial pose for one approach action run
    startActionlib();
    feedback_thread_ = boost::thread(boost::bind(&ManipulationController::publishFeedback, this));
  }
  virtual ~ManipulationController()
  {
    if (feedback_thread_.joinable())
    {
      feedback_thread_.interrupt();
      feedback_thread_.join();
    }

    action_server_->shutdown();
    delete action_server_;
  }

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

#ifndef __MOVE_CONTROLLER__
#define __MOVE_CONTROLLER__

#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>

namespace cartesian_controllers{

class MoveController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                                                        pr2_cartesian_controllers::MoveFeedback,
                                                                        pr2_cartesian_controllers::MoveResult>
{
private:
  // Actionlib
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();
  void publishFeedback();
  bool getDesiredJointPositions(geometry_msgs::PoseStamped pose, KDL::JntArray &joint_positions);
  bool finished_acquiring_goal_;

  // Controller values
  KDL::Frame pose_reference_;
  KDL::JntArray desired_joint_positions_;
  double velocity_gain_;
  double max_allowed_error_, error_threshold_;
  std::string ik_service_name_, ik_info_service_name_;

  // ROS
  ros::Publisher target_pub_, current_pub_;

public:
  MoveController() : ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                        pr2_cartesian_controllers::MoveFeedback,
                                        pr2_cartesian_controllers::MoveResult>()
  {
    if(!loadParams())
    {
      ros::shutdown();
      exit(0);
    }

    finished_acquiring_goal_ = false;
    startActionlib();
    target_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_target", 1);
    current_pub_ = nh_.advertise<visualization_msgs::Marker>("move_controller_current", 1);
    feedback_thread_ = boost::thread(boost::bind(&MoveController::publishFeedback, this));
  }
  virtual ~MoveController()
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

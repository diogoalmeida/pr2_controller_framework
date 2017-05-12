#ifndef __MOVE_CONTROLLER__
#define __MOVE_CONTROLLER__

#include <pr2_cartesian_controllers/MoveAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>

namespace cartesian_controllers{

/**
  Implementes a cartesian move controller.
**/
class MoveController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::MoveAction,
                                                                        pr2_cartesian_controllers::MoveFeedback,
                                                                        pr2_cartesian_controllers::MoveResult>
{
private:
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();
  void publishFeedback();

  /**
    Uses the pr2 inverse kinematics service to get the desired joint positions for the given pose.

    @param pose The desired cartesian pose
    @param joint_positions The joint positions that will allow the end-effector to attain the desired pose.
    @param joint_names Vector of joint names for which we have a reference position.

    @return True if the service provides a valid response. False otherwise.
  **/
  bool getDesiredJointPositions(const geometry_msgs::PoseStamped &pose, KDL::JntArray &joint_positions, std::vector<std::string> &joint_names);

  /**
    Get the desired reference position by namespace

    @param joint_name The query joint name.

    @return The reference position for the joint value.
  **/
  double getDesiredPosition(const std::string &joint_name);

private:
  bool finished_acquiring_goal_;
  // Controller values
  KDL::Frame pose_reference_;
  KDL::JntArray desired_joint_positions_;
  double velocity_gain_;
  double max_allowed_error_, error_threshold_;
  std::vector<std::string> ik_service_name_;
  std::vector<std::string> ik_info_service_name_;
  int arm_index_;

  // ROS
  ros::Publisher target_pub_, current_pub_;

public:
  MoveController();
  virtual ~MoveController();

  // Control topic: meant to be called in the realtime loop
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

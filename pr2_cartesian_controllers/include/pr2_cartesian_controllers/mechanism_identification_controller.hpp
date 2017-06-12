#ifndef __MECHANISM_IDENTIFICATION_CONTROLLER__
#define __MECHANISM_IDENTIFICATION_CONTROLLER__

#include <pr2_cartesian_controllers/MechanismIdentificationAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>
#include <pr2_algorithms/mechanism_identification/adaptive_velocity_controller.hpp>
#include <pr2_algorithms/mechanism_identification/kalman_filter.hpp>
#include <utils/TwistController.hpp>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <limits>

namespace cartesian_controllers{

/**
  Implements the mechanism_identification controller.
**/
class MechanismIdentificationController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::MechanismIdentificationAction,
                                                                                pr2_cartesian_controllers::MechanismIdentificationFeedback,
                                                                                pr2_cartesian_controllers::MechanismIdentificationResult>
{
private:
  /**
    Computes the skew-symmetric matrix of the provided vector.

    @param v The vector that will be used to build the skew-symmetric matrix.
    @return The computed skew-symmetric matrix.
  **/
  Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);

  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

private:
  // estimator
  manipulation_algorithms::KalmanEstimator estimator_;
  // controller
  manipulation_algorithms::AdaptiveController adaptive_controller_;
  manipulation_algorithms::ECTSController ects_controller_;

  bool has_initial_, finished_acquiring_goal_, use_estimates_;
  int rod_arm_, surface_arm_;
  double vd_amp_, vd_freq_, wd_amp_, wd_freq_, goal_force_, rod_length_;
  std::vector<KDL::Frame> eef_to_grasp_;
  Eigen::Affine3d pc_, p1_, p2_;
  ros::Publisher pc_pub_, p1_pub_, p2_pub_, r1_pub_, r2_pub_, wrench2_pub_;
  std::vector<double> comp_gains_;
  ros::Time elapsed_;

  boost::shared_ptr<TwistController> twist_controller_;

public:
  MechanismIdentificationController();
  virtual ~MechanismIdentificationController();
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

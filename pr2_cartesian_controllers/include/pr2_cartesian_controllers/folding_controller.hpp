#ifndef __FOLDING_CONTROLLER__
#define __FOLDING_CONTROLLER__

#include <pr2_cartesian_controllers/FoldingControllerAction.h>
#include <pr2_cartesian_controllers/controller_template.hpp>
#include <pr2_algorithms/folding_assembly_controller.hpp>
#include <pr2_algorithms/folding_assembly_estimator.hpp>
#include <utils/TwistController.hpp>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <limits>

namespace cartesian_controllers{

/**
  Implements the folding assembly controller.
**/
class FoldingController : public cartesian_controllers::ControllerTemplate<pr2_cartesian_controllers::FoldingControllerAction,
                                                                                pr2_cartesian_controllers::FoldingControllerFeedback,
                                                                                pr2_cartesian_controllers::FoldingControllerResult>
{
private:
  /**
    Computes the skew-symmetric matrix of the provided vector.

    @param v The vector that will be used to build the skew-symmetric matrix.
    @return The computed skew-symmetric matrix.
  **/
  Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);

  /**
    Fills a marker with the given initial and end point. Clears existing points.

    @param initial_point Initial marker point.
    @param final_point Final marker point.
    @param marker The marker object.
  **/
  void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker);
  virtual void publishFeedback();
  virtual void goalCB();
  virtual void preemptCB();
  virtual bool loadParams();

private:
  // estimator
  manipulation_algorithms::FoldingAssemblyEstimator estimator_;
  // controller
  manipulation_algorithms::FoldingAssemblyController controller_;

  bool has_initial_, finished_acquiring_goal_, use_estimates_;
  int rod_arm_, surface_arm_;
  double goal_p_, goal_theta_, goal_force_, rod_length_;
  std::vector<KDL::Frame> eef_to_grasp;
  Eigen::Affine3d pc_;
  ros::Publisher pc_publisher_;

  boost::shared_ptr<TwistController> twist_controller_;

public:
  FoldingController();
  virtual ~FoldingController();
  virtual sensor_msgs::JointState updateControl(const sensor_msgs::JointState &current_state, ros::Duration dt);
};
}

#endif

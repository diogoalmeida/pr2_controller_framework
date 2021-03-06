#ifndef __TWIST_CONTROLLER__
#define __TWIST_CONTROLLER__

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <iostream>

namespace cartesian_controllers{

/**
  Implements a pose controller that outputs a control twist.
**/
class TwistController{
public:
  TwistController(const Eigen::Matrix<double, 6, 1> &twist_gains);
  ~TwistController();

  /**
    Return a twist proportional to the pose error between the two given frames.

    @param current The present frame.
    @param reference The reference frame.
    @return A value proportional to the error "reference - current".
  **/
  KDL::Twist computeError(const KDL::Frame &current, const KDL::Frame &reference);
private:
  Eigen::Matrix<double, 6, 1> gains_;

  /**
    Computes the angle and axis rotation required to rotate v1 along v2.

    @param v1
    @param v2
    @return The angle axis rotation.
  **/
  Eigen::AngleAxisd getAngleAxis(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
};
}

#endif

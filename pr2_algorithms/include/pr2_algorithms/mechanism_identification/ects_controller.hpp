#ifndef __ECTS_CONTROL_ALGORITHM__
#define __ECTS_CONTROL_ALGORITHM__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/thread.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 7> Matrix67d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 6, 14> MatrixECTSr;
typedef Eigen::Matrix<double, 12, 14> MatrixECTS;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;

  /**
    Class that implements an extended cooperative task space (ECTS) control algorithm.

    Hardcoded dimensions to fit two 7 DOF manipulators.
  **/
  class ECTSController : public AlgorithmBase
  {
  public:
    ECTSController(const KDL::Chain &chain_1, const KDL::Chain &chain_2);
    ~ECTSController();

    virtual bool getParams(const ros::NodeHandle &n);

    /**
      Computes the ECTS reference joint velocities for the two manipulators.

      @param r_i The virtual sticks connecting the manipulators end-effectos to the object reference points.
      @param q_dot_i The manipulators' joint velocities.
      @param twist_a The commanded absolute motion twist.
      @param twist_r The commanded relative motion twist.
      @return The 14 dimensional joint velocities vector.
    **/
    Vector14d control(const Vector3d &r_1, const Vector3d &r_2, const KDL::JntArray &q1, const KDL::JntArray &q2, const Vector6d &twist_a, const Vector6d &twist_r);

    /**
      Adds a task relevant direction, u. The controller will try to optimize
      its velocity transmission ratio. Adding multiple u's will make the controller
      try to optimize the motion task compatibility index defined as

      \f$c_m = \sum_{i=1}^{n} \alpha_{m_i}^{\pm 2}\f$

      where

      \f$\alpha_{m_i} = [u_{m_i}^\top (J_{E_r}J_{E_r}^\top)^{-1}u_{m_i}]^{-1/2}\f$

      @param u The \f$u_{m_i}\f$ to be added.
    **/
    void addOptimizationDirection(const Vector6d &u);
  private:
    double alpha_, damping_;
    Matrix12d K_;
    int beta_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_1_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_2_;
    std::vector<Vector6d> u_list_;
    KDL::JntArray q1_, q2_;
    Vector3d r_1_, r_2_;

    /**
      Compute the velocity transmission ratio along the direction u.

      @param J The relative jacobian of the system.
      @param u The direction along which to measure the velocity transmission ratio.
    **/
    double computeTransmissionRatio(const MatrixECTSr &J, const Vector6d &u);

    /**
      Computes the relative jacobian that maps joints to relative task space twists.

      @return The relative jacobian.
    **/
    MatrixECTSr computeRelativeJacobian(const KDL::JntArray &q1, const KDL::JntArray &q2);

    /**
      Computes the task compatibility measure \f$c_m = \sum_{i=1}^n \alpha_{m_i}^{\pm2}\f$.
    **/
    double computeTaskCompatibility(const MatrixECTSr &J);

    Vector14d computeNullSpaceTask();
  };
}
#endif

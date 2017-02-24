#ifndef __MANIPULATION_EKF__
#define __MANIPULATION_EKF__
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <pr2_algorithms/algorithm_base.hpp>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
/**
  Class that implements an extended kalman filter observer to estimate
  the state of the dexterous manipulation controller.
*/
class ManipulationEKF : public AlgorithmBase
{
public:
  ManipulationEKF();
  ~ManipulationEKF();

  /**
    Initialize the EKF observer.

    The observer will keep estimates for the contact point between grasped object and surface,
    the angle between the object and the surface place, and the contact force.

    @param init_x The initial estimate for the system state: x = [x_c, theta_c, f_c]
  */
  void initialize(const Eigen::VectorXd &init_x);

  /**
    Update the EKF estimate.
    Updates the EKF estimate based on the current actuation commands and measured force, torque and end-effector orientation.

    @param u The commanded system imput.
    @param y The measurements vector.
    @param x_e The current end-effector state: [x_e, y_e, theta_e].
    @param dt The time-step since the last estimate update
    @return The new estimate values
  */
  Eigen::VectorXd estimate(const Eigen::Vector3d &u, const Eigen::VectorXd &y, const Eigen::Vector3d &x_e, const double dt);

  /**
    Obtain the parameters relevant to the estimator from the parameter server.
    The method will fetch the Q and R covariance matrices from the server.

    @param n The nodehandle that will be used to query the parameter server
    @return False in case of error
  */
  bool getParams(const ros::NodeHandle &n);

private:
  Eigen::VectorXd x_hat_;
  Eigen::MatrixXd P_, Q_, R_;
  double k_s_, theta_o_;

  void initializeMatrices(int dim, Eigen::MatrixXd &A, Eigen::MatrixXd &C, Eigen::MatrixXd &G, Eigen::MatrixXd &I, Eigen::MatrixXd &K, Eigen::MatrixXd &P);
  void computeA(Eigen::MatrixXd &A, const double y_e_dot, const double theta_e_dot, const double x_e, const double x_c, const double theta_c, const double f_c_y, const double f_c_x, const double k_s);
  void computeC(Eigen::MatrixXd &C, const double x_e, const double x_c, const double theta_c, const double f_c_y, const double f_c_x, const double k_s);
  void computeG(Eigen::MatrixXd &G, const double x_e, const double x_c, const double theta_c, const double theta_e, const double f_c_x, const double f_c_y, const double k_s);
};
}
#endif

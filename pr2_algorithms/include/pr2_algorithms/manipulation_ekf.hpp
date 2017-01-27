#ifndef __MANIPULATION_EKF__
#define __MANIPULATION_EKF__
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
/**
  Class that implements an extended kalman filter observer to estimate
  the state of the dexterous manipulation controller.
*/
class ManipulationEKF
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
  void initialize(const Eigen::Vector3d &init_x);

  /**
    Update the EKF estimate.
    Updates the EKF estimate based on the current actuation commands and measured force, torque and end-effector orientation.

    @param u The commanded system imput.
    @param y The measurements vector.
    @param x_e The current end-effector state: [x_e, y_e, theta_e].
    @param dt The time-step since the last estimate update
    @return The new estimate values
  */
  Eigen::Vector3d estimate(const Eigen::Vector3d &u, const Eigen::Vector3d &y, const Eigen::Vector3d &x_e, const double dt);

  /**
    Obtain the parameters relevant to the estimator from the parameter server.
    The method will fetch the Q and R covariance matrices from the server.

    @param n The nodehandle that will be used to query the parameter server
    @return False in case of error
  */
  bool getParams(const ros::NodeHandle &n);

private:
  Eigen::Vector3d x_hat_;
  Eigen::Matrix3d P_, Q_, R_;
  double k_s_, theta_o_;

  /**
    Initialize a 3x3 matrix from values obtained from the ros parameter
    server.

    @param M The matrix to be initialized
    @param configName The parameter server location
    @param n The ros nodehandle used to query the parameter server

    @return True for success, False otherwise
  */
  bool parseMatrixData(Eigen::Matrix3d &M, const std::string configName, const ros::NodeHandle &n);

  /**
    Fill in a 3x3 matrix with the given values.

    @param M The matrix to be filled in
    @param vals A vector with the values to fill in
  */
  void initializeEigenMatrix(Eigen::Matrix3d &M, const std::vector<double> vals);
};
}
#endif

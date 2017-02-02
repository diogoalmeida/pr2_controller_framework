#ifndef __MANIPULATION_CONTROL_ALGORITHM__
#define __MANIPULATION_CONTROL_ALGORITHM__
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>
#include <stdexcept>

namespace manipulation_algorithms{
  /**
    Class that implements the dexterous manipulation control algorithm.
  */
  class ManipulationAlgorithm
  {
  public:
    ManipulationAlgorithm();
    ~ManipulationAlgorithm();

    /**
      Compute the control values for the system.

      @param x_d The desired control goal.
      @param x_c The current system state.
      @param x_e The end-effector state.
      @return The computed control values
    */
    Eigen::Vector3d compute(const Eigen::Vector3d &x_d, const Eigen::Vector3d &x_c, const Eigen::Vector3d &x_e);

    /**
      Obtain the parameters relevant to the controller from the parameter server.

      @param n The nodehandle that will be used to query the parameter server
      @return False in case of error
    */
    bool getParams(const ros::NodeHandle &n);

  private:
    Eigen::Matrix3d Gamma_;
    double k_s_, max_command_x_, max_command_y_, max_command_theta_;

    /**
      Compute the inverse of the task jacobian from the system state.

      @param x_e The end-effector horizontal translation
      @param x_c The contact point horizontal translation
      @param theta_c The object orientation w.r.t the surface frame
      @return The inverse of the task jacobian
    */
    Eigen::Matrix3d computeInvG(const double x_e, const double x_c, const double theta_c);

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

    /**
      Saturates a control output

      @param original The computed output
      @param max The maximum allowed absolute value for the computed output
      @return The original, if abs(original) <= max, sign(original)*max otherwise
    */
    double saturateOutput(const double original, const double max);
  };
}
#endif

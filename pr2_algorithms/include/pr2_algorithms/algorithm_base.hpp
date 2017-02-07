#ifndef __ALGORITHM_BASE__
#define __ALGORITHM_BASE__
#include <Eigen/Dense>
#include <ros/ros.h>
#include <math.h>
#include <stdexcept>

namespace manipulation_algorithms{
  /**
    Class that defines an algorithm base.
  */
  class AlgorithmBase
  {
  public:
    AlgorithmBase();
    virtual ~AlgorithmBase();

    /**
      Obtain the parameters relevant to the controller from the parameter server.

      @param n The nodehandle that will be used to query the parameter server
      @return False in case of error
    */
    virtual bool getParams(const ros::NodeHandle &n) = 0;

  protected:
    /**
      Initialize a nxb matrix from values obtained from the ros parameter
      server.

      @param M The matrix to be initialized
      @param configName The parameter server location
      @param n The ros nodehandle used to query the parameter server

      @return True for success, False otherwise
    */
    virtual bool parseMatrixData(Eigen::MatrixXd &M, const std::string configName, const ros::NodeHandle &n);

    /**
      Fill in a nxn matrix with the given values.

      @param M The matrix to be filled in. Will be set to the size nxn.
      @param vals A vector with the values to fill in
    */
    virtual void initializeEigenMatrix(Eigen::MatrixXd &M, const std::vector<double> vals);

    /**
      Saturates a control output

      @param original The computed output
      @param max The maximum allowed absolute value for the computed output
      @return The original, if abs(original) <= max, sign(original)*max otherwise
    */
    virtual double saturateOutput(const double original, const double max);
  };
}
#endif

#include <pr2_algorithms/algorithm_base.hpp>

namespace manipulation_algorithms {

  AlgorithmBase::AlgorithmBase() {}
  AlgorithmBase::~AlgorithmBase() {}

  bool AlgorithmBase::parseMatrixData(Eigen::MatrixXd &M, const std::string configName, const ros::NodeHandle &n)
  {
    std::vector<double> vals;

    if(n.hasParam(configName.c_str()))
    {
      if(n.hasParam((configName + std::string("/data").c_str())))
      {
        n.getParam((configName + std::string("/data")).c_str(), vals);
        initializeEigenMatrix(M, vals);
      }
      else
      {
        ROS_ERROR("Matrix definition %s has no data values (%s)! Shutting down..."
        , configName.c_str(), (configName + std::string("/data")).c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Configuration name %s does not exist", configName.c_str());
      return false;
    }

    return true;
  }


  void AlgorithmBase::initializeEigenMatrix(Eigen::MatrixXd &M, const std::vector<double> vals)
  {
    // TODO: Checks
    int size = std::sqrt(vals.size());

    ROS_INFO("Initializing size %d", size);

    M = Eigen::MatrixXd(size, size);

    for(int i = 0; i < size; i++)
    {
      for(int j = 0; j < size; j++)
      {
        M(i,j) = vals[i*size + j];
      }
    }
  }

  double AlgorithmBase::saturateOutput(const double original, const double max)
  {
    if (std::abs(original) > max)
    {
      if (original < 0)
      {
        return -max;
      }

      return max;
    }

    return original;
  }
}

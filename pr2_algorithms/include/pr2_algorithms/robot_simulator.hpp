#ifndef __ROBOT_SIMULATOR__KIN__
#define __ROBOT_SIMULATOR__KIN__
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>

/**
  Class that maintains a kinematic simulation of the robot loaded in the robot_description parameter
  in the ROS parameter server. It publishes the TF transforms through the robot state publisher.
**/
class RobotSimulator
{
public:
  RobotSimulator(double frequency);

  /**
    Initializes the kinematic chain connecting base_link to end_effector_link.
    The end_effector_link name will be used to index this chain.

    @param base_link The kinematic chain base link.
    @param end_effector_link The kinematic chain end-effector.
    @param desired_pose: Initialization pose for the end-effector
    @return True for a successful initialization; False in case of failure.
  **/
  bool initKinematicChain(const std::string &base_link, const std::string &end_effector_link, const std::vector<double> &desired_pose);

  /**
    Set the desired chain to a pose.
  **/
  bool setPose(const std::string &end_effector_link, const std::vector<double> &desired_pose);

  /**
    Get an initialized kinematic chain.

    @param end_effector_link The name of the chain's end-effector.
    @param chain The variable where to store the chain.
    @return False for failure in getting the chain.
  **/
  bool getKinematicChain(const std::string &end_effector_link, KDL::Chain &chain);

  /**
    Gets the pose of the end-effector.
  **/
  bool getPose(const std::string &end_effector_link, KDL::Frame &pose);

  /**
    Gets the current kinematic chain joint state
  **/
  bool getJointState(const std::string &end_effector_link, KDL::JntArray &q);

  /**
    Set the current joint velocities for the joint chain
  **/
  bool setJointVelocities(const std::string &end_effector_link, const Eigen::VectorXd &joint_velocities);
private:
  boost::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_publisher_;
  KDL::Tree tree_;

  std::vector<std::string> end_effector_link_name_;
  std::vector<KDL::Chain> chain_;
  std::vector<KDL::JntArrayVel> joint_state_;
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos_recursive> > fk_solver_;
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos_LMA> > ik_solver_;
  std::map<std::string, double> joint_positions_;
  std::vector<Eigen::VectorXd> current_joint_velocities_;
  boost::mutex velocities_mutex_;
  boost::thread sim_thread_;
  double frequency_;

  /**
    Gets a KDL frame from a vector with a 7 dimensional pose description.

    @param pose_in Vector with a 3 dimensional position and a 4 dimensional quaternion representing orientation.
    @return The KDL frame that represents the same pose.
  **/
  KDL::Frame getKDLPose(const std::vector<double> &pose_in);

  /**
    Simulation thread.
  **/
  void simulation();

  /**
    Apply the given joint velocities to the kinematic chain indexed by the end-effector link.

    @param joint_velocities The joint velocities vector.
    @param end_effector_link The kinematic chain end-effector.
    @param dt Time step.
    @return False in case of failure.
  **/
  bool applyJointVelocities(const Eigen::VectorXd &joint_velocities, const std::string &end_effector_link, double dt);
};

#endif
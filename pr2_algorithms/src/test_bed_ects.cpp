#include <ros/ros.h>
#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>
#include <pr2_algorithms/TestBedECTSFeedback.h>
#include <pr2_cartesian_controllers/MechanismIdentificationAction.h>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pr2_algorithms/robot_simulator.hpp>
// #include <tf/Quaternion.h>
// #include <tf/Matrix3x3.h>
#include <cstdlib>

using namespace manipulation_algorithms;

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d ret;

  ret << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;

  return ret;
}

typedef Eigen::Matrix<double, 12, 14> Matrix1214d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");
  ros::NodeHandle n("~");
  pr2_algorithms::TestBedECTSFeedback feedback_msg;
  urdf::Model model;
  KDL::Tree tree;
  std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2), p_eig(2);
  std::vector<KDL::Frame> eef_grasp_kdl(2), p(2), eef_to_grasp(2);
  Eigen::Vector3d p1, p2, pc;
  Vector12d command_twist = Vector12d::Zero();
  Vector14d out = Vector14d::Zero();
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  tf::TransformListener listener;
  ros::Rate r(100);
  std::vector<double> pose_vector;
  KDL::Frame pose_frame;
  double max_time, epsilon;
  RobotSimulator simulator(100);

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedECTSFeedback>("/test_bed/feedback", 1);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/sim_joint_states", 1);

  ROS_INFO("Setting initial joint state");
  if(!n.getParam("left_arm_pose", pose_vector))
  {
    ROS_ERROR("Missing left_arm_pose");
    return false;
  }
  simulator.initKinematicChain("torso_lift_link", "l_wrist_roll_link", pose_vector);

  if(!n.getParam("right_arm_pose", pose_vector))
  {
    ROS_ERROR("Missing right_arm_pose");
    return false;
  }
  simulator.initKinematicChain("torso_lift_link", "r_wrist_roll_link", pose_vector);

  KDL::Chain l_chain, r_chain;
  simulator.getKinematicChain("l_wrist_roll_link", l_chain);
  simulator.getKinematicChain("r_wrist_roll_link", r_chain);
  manipulation_algorithms::ECTSController controller(l_chain, r_chain);
  controller.getParams(n);

  if (argc > 1 && atof(argv[1]) > 0)
  {
    max_time = atof(argv[1]);
  }
  else
  {
    max_time = 10;
  }

  while(ros::Time::now() == ros::Time(0)) // wait for simulation time
  {
    sleep(0.1);
  }

  geometry_msgs::PoseStamped pose_in, pose_out;
  std::string ft_frame_id[] = {"l_gripper_tool_frame", "r_gripper_tool_frame"};
  std::string end_effector_link[] = {"l_wrist_roll_link", "r_wrist_roll_link"};

  bool done_acquiring_tf = false;
  while (!done_acquiring_tf && ros::ok())
  {
    try
    {
      // get the relationship between kinematic chain end-effector and
      // tool-tip (grasping point)
      for (int i = 0; i < 2; i++)
      {
        pose_in.header.frame_id = ft_frame_id[i];
        pose_in.header.stamp = ros::Time(0); // get latest available

        pose_in.pose.position.x = 0;
        pose_in.pose.position.y = 0;
        pose_in.pose.position.z = 0;

        pose_in.pose.orientation.x = 0;
        pose_in.pose.orientation.y = 0;
        pose_in.pose.orientation.z = 0;
        pose_in.pose.orientation.w = 1;
        listener.transformPose(end_effector_link[i], pose_in, pose_out);
        tf::poseMsgToKDL(pose_out.pose, eef_to_grasp[i]);
        done_acquiring_tf = true;
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF exception: %s", ex.what());
    }
    sleep(1);
  }

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();
  pc = Eigen::Vector3d::Zero();
  double vd_freq = 0.1, wd_freq = 0.05, vd_amp = 0.0, wd_amp = 0.06;
  bool acquired_dof = false;
  ROS_INFO("Starting simulation");
  Eigen::Vector3d rotational_dof_ground, translational_dof_ground;
  while (elapsed.toSec() < max_time && ros::ok())
  {
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;

    for (int arm = 0; arm < 2; arm++) // Compute forward kinematics and convert to grasp frame
    {
      simulator.getPose(end_effector_link[arm], p[arm]);
      eef_grasp_kdl[arm] = p[arm]*eef_to_grasp[arm];
      tf::transformKDLToEigen(p[arm], p_eig[arm]);
      tf::transformKDLToEigen(eef_to_grasp[arm], eef_to_grasp_eig[arm]);
      tf::transformKDLToEigen(eef_grasp_kdl[arm], grasp_point_frame[arm]);
    }

    for (int i = 0; i < 3; i++)
    {
      p1[i] = p[0].p[i];
      p2[i] = p[1].p[i];
    }

    if (!acquired_dof)
    {
      rotational_dof_ground  = grasp_point_frame[1].matrix().block<3,1>(0,1);
      translational_dof_ground = grasp_point_frame[1].matrix().block<3,1>(0,0);
      // acquired_dof = true;
    }
    pc = p1 + 0.1*grasp_point_frame[0].matrix().block<3,1>(0,0);
    command_twist.block<3,1>(6,0) = vd_amp*sin(2*M_PI*vd_freq*elapsed.toSec())*translational_dof_ground;
    command_twist.block<3,1>(9,0) = wd_amp*sin(2*M_PI*wd_freq*elapsed.toSec())*rotational_dof_ground;

    bool use_nullspace = false;
    Eigen::Matrix<double, 12, 1> transmission_direction;

    if (use_nullspace)
    {
      controller.clearOptimizationDirections();
      transmission_direction = Eigen::Matrix<double, 12, 1>::Zero();
      transmission_direction.block<3,1>(6,0) = translational_dof_ground;
      // controller.addOptimizationDirection(transmission_direction);
      transmission_direction = Eigen::Matrix<double, 12, 1>::Zero();
      transmission_direction.block<3,1>(9,0) = rotational_dof_ground;
      controller.addOptimizationDirection(transmission_direction);
      controller.setNullspaceGain(0.01);
      // std::cout << controller.getTaskCompatibility() << std::endl;
    }

    KDL::JntArray l_q, r_q;
    simulator.getJointState(end_effector_link[0], l_q);
    simulator.getJointState(end_effector_link[1], r_q);

    out = controller.control(pc - p1, pc - p2, l_q, r_q, command_twist.block<6,1>(0,0), command_twist.block<6,1>(6,0));

    // std::cout << out << std::endl << std::endl;
    simulator.setJointVelocities(end_effector_link[0], out.block<7, 1>(0, 0));
    simulator.setJointVelocities(end_effector_link[1], out.block<7, 1>(7, 0));

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    ros::spinOnce();
    r.sleep();
  }
}

#include <ros/ros.h>
#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>
#include <pr2_algorithms/TestBedECTSFeedback.h>
#include <pr2_cartesian_controllers/MechanismIdentificationAction.h>
#include <urdf/model.h>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
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

KDL::Frame getKDLPose(const std::vector<double> &pose_in)
{
  tf::Quaternion quat(pose_in[3], pose_in[4], pose_in[5], pose_in[6]);
  tf::Matrix3x3 mat(quat);
  KDL::Vector vec(pose_in[0], pose_in[1], pose_in[2]);
  KDL::Rotation rot;
  double roll, pitch, yaw;
  
  mat.getRPY(roll, pitch, yaw);
  rot = rot.RPY(roll, pitch, yaw);
  
  std::cout << roll << " " << pitch << " " << yaw << std::endl;
  
  KDL::Frame frame(rot, vec);
  
  return frame;
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
  std::vector<KDL::Chain> chain(2);
  std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver(2);
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos_recursive> > fk_solver(2);
  std::vector<boost::shared_ptr<KDL::ChainIkSolverPos_LMA> > ik_solver(2);
  std::vector<KDL::Jacobian> jacobian(2, KDL::Jacobian(7));
  std::vector<KDL::JntArrayVel> joint_state(2, KDL::JntArrayVel(7));
  std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2);
  std::vector<KDL::Frame> eef_grasp_kdl(2), p(2), eef_to_grasp(2);
  Eigen::Vector3d p1, p2, pc;
  Vector12d command_twist;
  Vector14d out;
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  tf::TransformListener listener;
  ros::Rate r(100);
  std::vector<double> pose_vector;
  KDL::Frame pose_frame;
  double max_time, epsilon;

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedECTSFeedback>("/test_bed/feedback", 1);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/sim_joint_states", 1);
  if(!model.initParam("/robot_description")){
      ROS_ERROR("ERROR getting robot description (/robot_description)");
      return -1;
  }
  
  ROS_INFO("Loading URDF model");
  kdl_parser::treeFromUrdfModel(model, tree);
  
  tree.getChain("torso_lift_link", "l_wrist_roll_link", chain[0]);
  joint_state[0].q.resize(chain[0].getNrOfJoints());
  joint_state[0].qdot.resize(chain[0].getNrOfJoints());
  jac_solver[0].reset(new KDL::ChainJntToJacSolver(chain[0]));
  fk_solver[0].reset(new KDL::ChainFkSolverPos_recursive(chain[0]));
  ik_solver[0].reset(new KDL::ChainIkSolverPos_LMA(chain[0]));
  
  tree.getChain("torso_lift_link", "r_wrist_roll_link", chain[1]);
  joint_state[1].q.resize(chain[1].getNrOfJoints());
  joint_state[1].qdot.resize(chain[1].getNrOfJoints());
  jac_solver[1].reset(new KDL::ChainJntToJacSolver(chain[1]));
  fk_solver[1].reset(new KDL::ChainFkSolverPos_recursive(chain[1]));
  ik_solver[1].reset(new KDL::ChainIkSolverPos_LMA(chain[1]));
  
  ROS_INFO("Setting initial joint state");
  if(!n.getParam("left_arm_pose", pose_vector))
  {
    ROS_ERROR("Missing left_arm_pose");
    return false;
  }
  
  pose_frame = getKDLPose(pose_vector);
  ik_solver[0]->CartToJnt(joint_state[0].q, pose_frame, joint_state[0].q);
  
  if(!n.getParam("right_arm_pose", pose_vector))
  {
    ROS_ERROR("Missing right_arm_pose");
    return false;
  }
  
  pose_frame = getKDLPose(pose_vector);
  ik_solver[1]->CartToJnt(joint_state[1].q, pose_frame, joint_state[1].q);
  
  manipulation_algorithms::ECTSController controller(chain[0], chain[1]);
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
  
  std::vector<std::string> joint_names = {"_shoulder_pan_joint", "_shoulder_lift_joint", "_upper_arm_roll_joint", "_elbow_flex_joint", "_forearm_roll_joint", "_wrist_flex_joint", "_wrist_roll_joint"};
  sensor_msgs::JointState state;
  state.name.resize(14);
  state.position.resize(14);
  state.velocity.resize(14);
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
    state_pub.publish(state);
    sleep(1);
  }

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();
  pc = Eigen::Vector3d::Zero();
  double vd_freq = 0.1, wd_freq = 0.1, vd_amp = 0.03, wd_amp = 0.0;
  
  ROS_INFO("Starting simulation");
  Eigen::Vector3d rotational_dof_ground, translational_dof_ground;
  while (elapsed.toSec() < max_time && ros::ok())
  {
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;
    
    for (int arm = 0; arm < 2; arm++) // Compute forward kinematics and convert to grasp frame
    {
      fk_solver[arm]->JntToCart(joint_state[arm].q, p[arm]);
      eef_grasp_kdl[arm] = p[arm]*eef_to_grasp[arm];
      tf::transformKDLToEigen(eef_to_grasp[arm], eef_to_grasp_eig[arm]);
      tf::transformKDLToEigen(eef_grasp_kdl[arm], grasp_point_frame[arm]);
    }

    for (int i = 0; i < 3; i++)
    {
      p1[i] = p[0].p[i];
      p2[i] = p[1].p[i];
    }
    
    rotational_dof_ground  = grasp_point_frame[1].matrix().block<3,1>(0,1);
    translational_dof_ground = grasp_point_frame[1].matrix().block<3,1>(0,0);
    command_twist.block<3,1>(6,0) = vd_amp*sin(2*M_PI*vd_freq*elapsed.toSec())*translational_dof_ground;
    command_twist.block<3,1>(9,0) = wd_amp*sin(2*M_PI*wd_freq*elapsed.toSec())*rotational_dof_ground;
    
    bool use_nullspace = true;
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
      std::cout << controller.getTaskCompatibility() << std::endl;
    }
    
    out = controller.control(pc - p1, pc - p2, joint_state[0].q, joint_state[1].q, command_twist.block<6,1>(0,0), command_twist.block<6,1>(6,0));

    // std::cout << out << std::endl << std::endl;

    joint_state[0].qdot.data = out.block<7, 1>(0, 0);
    joint_state[0].q.data += joint_state[0].qdot.data*dt.toSec();
    joint_state[1].qdot.data = out.block<7, 1>(7, 0);
    joint_state[1].q.data += joint_state[1].qdot.data*dt.toSec();

    for (int i = 0; i < 7; i++)
    {
      state.name[i] = "r" + joint_names[i];
      state.name[i + 7] = "l" + joint_names[i];
      state.position[i] = joint_state[0].q.data[i];
      state.position[i + 7] = joint_state[1].q.data[i];
      state.velocity[i] = joint_state[0].qdot.data[i];
      state.velocity[i + 7] = joint_state[1].qdot.data[i];
    }

    prev_time = ros::Time::now();
    state_pub.publish(state);
    pub.publish(feedback_msg);
    ros::spinOnce();
    r.sleep();
  }
}

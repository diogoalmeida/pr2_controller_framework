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
#include <eigen_conversions/eigen_msg.h>
#include <pr2_algorithms/robot_simulator.hpp>
#include <pr2_algorithms/ECTSDebugAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>
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

double vd_freq = 0.0, wd_freq = 0.0, vd_amp = 0.0, wd_amp = 0.0;
ros::Time init_time, prev_time;
std::string end_effector_link[] = {"l_wrist_roll_link", "r_wrist_roll_link"};

boost::mutex cb_mutex;
boost::shared_ptr<actionlib::SimpleActionServer<pr2_algorithms::ECTSDebugAction> > action_server;

void goalCB()
{
  boost::mutex::scoped_lock lock(cb_mutex);
  boost::shared_ptr<const pr2_algorithms::ECTSDebugGoal> goal = action_server->acceptNewGoal();

  vd_freq = goal->vd_freq;
  wd_freq = goal->wd_freq;
  vd_amp = goal->vd_amp;
  wd_amp = goal->wd_amp;
  init_time = ros::Time::now();
  prev_time = ros::Time::now();
}

void preemptCB(RobotSimulator &simulator)
{
  Vector7d null = Vector7d::Zero();
  boost::mutex::scoped_lock lock(cb_mutex);
  simulator.setJointVelocities(end_effector_link[0], null);
  simulator.setJointVelocities(end_effector_link[1], null);
  action_server->setPreempted();
  ROS_WARN("ECTS controller preempted");
}

void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker)
{
  geometry_msgs::Point point;

  marker.points.clear();
  tf::pointEigenToMsg(initial_point, point);
  marker.points.push_back(point);
  tf::pointEigenToMsg(final_point, point);
  marker.points.push_back(point);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");
  ros::NodeHandle n("~");
  pr2_algorithms::TestBedECTSFeedback feedback_msg;
  urdf::Model model;
  KDL::Tree tree;
  Eigen::Affine3d pc_eig;
  std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2), p_eig(2);
  std::vector<KDL::Frame> eef_grasp_kdl(2), p(2), eef_to_grasp(2);
  Eigen::Vector3d p1, p2, pc;
  Vector12d command_twist = Vector12d::Zero();
  Vector14d out = Vector14d::Zero();
  ros::Duration dt, elapsed;
  tf::TransformListener listener;
  ros::Rate r(100);
  std::vector<double> pose_vector;
  KDL::Frame pose_frame;
  RobotSimulator simulator(100);
  double max_time, epsilon;
  visualization_msgs::Marker pc_marker, p1_marker, p2_marker, r1_marker, r2_marker, trans_marker, rot_marker;

  ros::Publisher pc_pub = n.advertise<visualization_msgs::Marker>("pc", 1);
  ros::Publisher p1_pub = n.advertise<visualization_msgs::Marker>("p1", 1);
  ros::Publisher p2_pub = n.advertise<visualization_msgs::Marker>("p2", 1);
  ros::Publisher r1_pub = n.advertise<visualization_msgs::Marker>("r1", 1);
  ros::Publisher r2_pub = n.advertise<visualization_msgs::Marker>("r2", 1);
  ros::Publisher trans_pub = n.advertise<visualization_msgs::Marker>("trans", 1);
  ros::Publisher rot_pub = n.advertise<visualization_msgs::Marker>("rot", 1);

  pc_marker.header.frame_id = "torso_lift_link";
  pc_marker.ns = std::string("mechanism_identification");
  pc_marker.type = pc_marker.SPHERE;
  pc_marker.action = pc_marker.ADD;
  pc_marker.scale.x = 0.01;
  pc_marker.scale.y = 0.01;
  pc_marker.scale.z = 0.01;
  pc_marker.lifetime = ros::Duration(0);
  pc_marker.frame_locked = false;
  pc_marker.color.r = 1.0;
  pc_marker.color.a = 1.0;
  p1_marker = pc_marker;
  p1_marker.id = 1;
  p2_marker = pc_marker;
  p2_marker.id = 2;
  p1_marker.color.r = 0.0;
  p2_marker.color.r = 0.0;
  p1_marker.color.g = 1.0;
  p2_marker.color.g = 1.0;
  r1_marker = pc_marker;
  r1_marker.id = 3;
  r1_marker.scale.y = 0.005;
  r1_marker.scale.z = 0.005;
  r1_marker.type = r1_marker.ARROW;
  r2_marker = r1_marker;
  r2_marker.id = 4;
  r2_marker.color = pc_marker.color;
  trans_marker = r2_marker;
  trans_marker.id = 5;
  trans_marker.color.r = 1.0;
  trans_marker.color.g = 0.0;
  trans_marker.scale.x = 0.01;
  trans_marker.scale.y = 0.01;
  trans_marker.scale.z = 0.01;
  rot_marker = trans_marker;
  rot_marker.color.r = 0.0;
  rot_marker.color.b = 1.0;

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

  action_server = boost::shared_ptr<actionlib::SimpleActionServer<pr2_algorithms::ECTSDebugAction> >(new actionlib::SimpleActionServer<pr2_algorithms::ECTSDebugAction>(n, "ects_debug", false));
  action_server->registerGoalCallback(&goalCB);
  action_server->registerPreemptCallback(boost::bind(&preemptCB, boost::ref(simulator)));
  action_server->start();

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
  bool acquired_dof = false;
  ROS_INFO("Starting simulation");
  Eigen::Vector3d rotational_dof_ground, translational_dof_ground;
  while (ros::ok())
  {
    ros::spinOnce();
    if (!action_server->isActive())
    {
      r.sleep();
      continue;
    }

    {
      boost::mutex::scoped_lock lock(cb_mutex);
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
      pc_eig.translation() = pc;
      pc_eig.linear() = grasp_point_frame[0].linear();
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

      tf::poseEigenToMsg(pc_eig, pc_marker.pose);
      tf::poseEigenToMsg(grasp_point_frame[0], p1_marker.pose);
      tf::poseEigenToMsg(grasp_point_frame[1], p2_marker.pose);
      getMarkerPoints(pc_eig.translation(), pc_eig.translation() + 0.1*translational_dof_ground, trans_marker);
      getMarkerPoints(pc_eig.translation(), pc_eig.translation() + 0.1*rotational_dof_ground, rot_marker);

      pc_pub.publish(pc_marker);
      p1_pub.publish(p1_marker);
      p2_pub.publish(p2_marker);
      trans_pub.publish(trans_marker);
      rot_pub.publish(rot_marker);

      out = controller.control(pc - p1, pc - pc, l_q, r_q, command_twist.block<6,1>(0,0), command_twist.block<6,1>(6,0));

      // std::cout << out << std::endl << std::endl;
      simulator.setJointVelocities(end_effector_link[0], out.block<7, 1>(0, 0));
      simulator.setJointVelocities(end_effector_link[1], out.block<7, 1>(7, 0));
    }

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    r.sleep();
  }
}

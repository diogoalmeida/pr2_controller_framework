#include <ros/ros.h>
#include <pr2_algorithms/mechanism_identification/ects_controller.hpp>
#include <pr2_algorithms/TestBedECTSFeedback.h>
#include <pr2_cartesian_controllers/MechanismIdentificationAction.h>
#include <urdf/model.h>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <sensor_msgs/JointState.h>
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

  ros::NodeHandle n;
  pr2_algorithms::TestBedECTSFeedback feedback_msg;
  urdf::Model model;
  KDL::Tree tree;
  std::vector<KDL::Chain> chain(2);
  std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver(2);
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos_recursive> > fk_solver(2);
  std::vector<KDL::Jacobian> jacobian(2, KDL::Jacobian(7));
  std::vector<KDL::JntArrayVel> joint_state(2, KDL::JntArrayVel(7));
  std::vector<KDL::Frame> p(2);
  Eigen::Vector3d p1, p2, pc;
  Vector12d command_twist;
  Vector14d out;
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  ros::Rate r(1000);
  double max_time, epsilon;

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedECTSFeedback>("/test_bed/feedback", 1);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/sim_joint_states", 1);
  if(!model.initParam("/robot_description")){
      ROS_ERROR("ERROR getting robot description (/robot_description)");
      return -1;
  }

  kdl_parser::treeFromUrdfModel(model, tree);
  tree.getChain("torso_lift_link", "l_wrist_roll_link", chain[0]);
  joint_state[0].q.resize(chain[0].getNrOfJoints());
  joint_state[0].qdot.resize(chain[0].getNrOfJoints());
  jac_solver[0].reset(new KDL::ChainJntToJacSolver(chain[0]));
  fk_solver[0].reset(new KDL::ChainFkSolverPos_recursive(chain[0]));
  tree.getChain("torso_lift_link", "r_wrist_roll_link", chain[1]);
  joint_state[1].q.resize(chain[1].getNrOfJoints());
  joint_state[1].qdot.resize(chain[1].getNrOfJoints());
  jac_solver[1].reset(new KDL::ChainJntToJacSolver(chain[1]));
  fk_solver[1].reset(new KDL::ChainFkSolverPos_recursive(chain[1]));
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

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();
  pc = Eigen::Vector3d::Zero();

  std::vector<std::string> joint_names = {"_shoulder_pan_joint", "_shoulder_lift_joint", "_upper_arm_roll_joint", "_elbow_flex_joint", "_forearm_roll_joint", "_wrist_flex_joint", "_wrist_roll_joint"};
  sensor_msgs::JointState state;
  state.name.resize(14);
  state.position.resize(14);
  state.velocity.resize(14);

  while (elapsed.toSec() < max_time)
  {
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;

    jac_solver[0]->JntToJac(joint_state[0].q, jacobian[0]);
    fk_solver[0]->JntToCart(joint_state[0].q, p[0]);
    jac_solver[1]->JntToJac(joint_state[1].q, jacobian[1]);
    fk_solver[1]->JntToCart(joint_state[1].q, p[1]);

    for (int i = 0; i < 3; i++)
    {
      p1[i] = p[0].p[i];
      p2[i] = p[1].p[i];
    }

    command_twist = Vector12d::Zero();

    out = controller.control(pc - p1, pc - p2, joint_state[0].q, joint_state[1].q, command_twist.block<6,1>(0,0), command_twist.block<6,1>(6,0));

    std::cout << out << std::endl << std::endl;

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
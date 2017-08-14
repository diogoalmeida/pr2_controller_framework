#include <ros/ros.h>
#include <pr2_algorithms/mechanism_identification/kalman_filter.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr2_algorithms/robot_simulator.hpp>
#include <pr2_algorithms/TestKalmanFeedback.h>
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

ros::Time init_time, prev_time;
Vector6d measured_wrench_;

void getMarkerPoints(const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point, visualization_msgs::Marker &marker)
{
  geometry_msgs::Point point;

  marker.points.clear();
  tf::pointEigenToMsg(initial_point, point);
  marker.points.push_back(point);
  tf::pointEigenToMsg(final_point, point);
  marker.points.push_back(point);
}

void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  tf::wrenchMsgToEigen(msg->wrench, measured_wrench_);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");
  ros::NodeHandle n("~");
  Eigen::Affine3d pc_eig;
  std::vector<Eigen::Affine3d> eef_to_grasp_eig(2), grasp_point_frame(2), p_eig(2);
  std::vector<KDL::Frame> eef_grasp_kdl(2), p(2), eef_to_grasp(2);
  Eigen::Vector3d p1, p2, pc, eef1, eef2;
  Vector12d command_twist = Vector12d::Zero();
  Vector14d out = Vector14d::Zero();
  ros::Duration dt, elapsed;
  tf::TransformListener listener;
  ros::Rate r(100);
  KDL::Frame pose_frame;
  double max_time, epsilon;
  std::string ft_frame_id, ft_topic;
  pr2_algorithms::TestKalmanFeedback feedback_msg;
  visualization_msgs::Marker pc_marker;

  ros::Publisher pc_pub = n.advertise<visualization_msgs::Marker>("pc", 1);
  
  if(!n.getParam("frame_id", ft_frame_id))
  {
    ROS_ERROR("Missing frame id");
    return -1;
  }
  
  if(!n.getParam("wrench_topic", ft_topic))
  {
    ROS_ERROR("Missing wrench topic");
    return -1;
  }

  pc_marker.header.frame_id = ft_frame_id;
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

  ros::Subscriber wrench_sub = n.subscribe(ft_topic, 1, wrenchCallback);
  ros::Publisher pub = n.advertise<pr2_algorithms::TestKalmanFeedback>("/test_bed/feedback", 1);
  manipulation_algorithms::KalmanEstimator estimator;
  if (!estimator.getParams(n))
  {
    return -1;
  }
  estimator.initialize(Eigen::Vector3d::Zero());
  measured_wrench_ = Vector6d::Zero();
  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  pc = Eigen::Vector3d::Zero();
  bool acquired_dof = false;
  double trans_estimate = 0.0, rot_estimate = 0.0, normal_estimate = 0.0; // computation along these directions
  while (ros::ok())
  {
    ros::spinOnce();
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;

    pc = estimator.estimate(Eigen::Vector3d::Zero(), Vector6d::Zero(), Eigen::Vector3d::Zero(), measured_wrench_, dt.toSec());
    pc_eig.translation() = pc;

    tf::poseEigenToMsg(pc_eig, pc_marker.pose);
    pc_pub.publish(pc_marker);

    feedback_msg.computations.clear();
    trans_estimate = measured_wrench_[4]/measured_wrench_[2];
    rot_estimate = measured_wrench_[3]/measured_wrench_[2];
    normal_estimate = measured_wrench_[3]/measured_wrench_[1];

    feedback_msg.computations.push_back(trans_estimate);
    feedback_msg.computations.push_back(rot_estimate);
    feedback_msg.computations.push_back(normal_estimate);

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    r.sleep();
  }
}

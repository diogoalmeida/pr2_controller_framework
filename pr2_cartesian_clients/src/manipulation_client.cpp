#include <manipulation_client/manipulation_client.hpp>

namespace manipulation{

/*
  Get the client parameters and abort in case they are not found
*/
bool ManipulationClient::loadParams()
{
  if(!nh_.getParam("vision/surface_frame_name", surface_frame_name_))
  {
    ROS_ERROR("No surface frame name defined (vision/surface_frame_name)");
    return false;
  }

  if(!nh_.getParam("experiment/base_link_name", base_link_name_))
  {
    ROS_ERROR("No base link frame name defined (base_link_name)");
    return false;
  }

  if(!nh_.getParam("experiment/tool_frame_name", tool_frame_name_))
  {
    ROS_ERROR("No tool frame frame name defined (tool_frame_name)");
    return false;
  }

  if(!nh_.getParam("initialization/initial_pose_offset", initial_pose_offset_))
  {
    ROS_ERROR("No inital eef offset defined (initialization/initial_pose_offset)");
    return false;
  }

  if(!nh_.getParam("initialization/initial_approach_angle", initial_approach_angle_))
  {
    ROS_ERROR("No inital approach angle defined (initialization/initial_approach_angle)");
    return false;
  }

  if(!nh_.getParam("initialization/manipulation_action_name", manipulation_action_name_))
  {
    ROS_ERROR("No manipulation action name defined (initialization/manipulation_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/approach_action_name", approach_action_name_))
  {
    ROS_ERROR("No approach action name defined (initialization/approach_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/move_action_name", move_action_name_))
  {
    ROS_ERROR("No move action name defined (initialization/move_action_name)");
    return false;
  }

  if(!nh_.getParam("experiment/num_of_experiments", num_of_experiments_))
  {
    ROS_ERROR("No number of experiments defined (experiment/num_of_experiments)");
    return false;
  }

  if(!nh_.getParam("experiment/use_vision", use_vision_))
  {
    ROS_ERROR("Need to set if vision is to be used (experiment/use_vision)");
    return false;
  }

  if(!nh_.getParam("experiment/sim_mode", sim_mode_))
  {
    ROS_ERROR("Need to set sim_mode (experiment/sim_mode)");
    return false;
  }

  if(!nh_.getParam("initialization/gravity_compensation_service_name", gravity_compensation_service_name_))
  {
    ROS_ERROR("No number gravity compensation service name defined (initialization/gravity_compensation_service_name)");
    return false;
  }

  return true;
}

/*
  Initializes the robot pose based on tag information. Commands the initial
  approach (another action? here?). Monitors vision information to compute
  ground truth data, and registers feedback.
*/
void ManipulationClient::runExperiment()
{
  if(!loadParams())
  {
    ros::shutdown();
    return;
  }

  manipulation_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction>(manipulation_action_name_, true);
  approach_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction>(approach_action_name_, true);
  move_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction>(move_action_name_, true);

  ROS_INFO("%s client waiting for server", manipulation_action_name_.c_str());
  if(!manipulation_action_client_->waitForServer(ros::Duration(server_timeout_)))
  {
    ROS_ERROR("%s was not found. Aborting", manipulation_action_name_.c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("%s client waiting for server", approach_action_name_.c_str());
  if(!approach_action_client_->waitForServer(ros::Duration(server_timeout_)))
  {
    ROS_ERROR("%s was not found. Aborting", approach_action_name_.c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("%s client waiting for server", move_action_name_.c_str());
  if(!move_action_client_->waitForServer(ros::Duration(server_timeout_)))
  {
    ROS_ERROR("%s was not found. Aborting", move_action_name_.c_str());
    ros::shutdown();
    return;
  }

  if(use_vision_)
  {
    ROS_INFO("Cartesian client waiting for table pose");
    if(!waitForTablePose(ros::Duration(vision_timeout_)))
    {
      ROS_ERROR("Could not find table frame. Aborting");
      ros::shutdown();
      return;
    }
  }

  if(!sim_mode_)
  {
    gravity_compensation_client_ = nh_.serviceClient<std_srvs::Empty>(gravity_compensation_service_name_);

    // Zero the ft sensor readings
    std_srvs::Empty srv;

    if(!gravity_compensation_client_.call(srv))
    {
      ROS_ERROR("Error calling the gravity compensation server!");
      ros::shutdown();
      return;
    }
  }

  // At this point I have knowledge of the arm that I want to move (tool frame)
  // and I can compute the desired initial pose of the end-effector
  geometry_msgs::PoseStamped initial_eef_pose = getInitialEefPose();

  pr2_cartesian_controllers::MoveGoal move_goal;
  pr2_cartesian_controllers::GuardedApproachGoal approach_goal;
  pr2_cartesian_controllers::ManipulationControllerGoal manipulation_goal;

  ROS_INFO("Starting experiment!");
  //while(experiment_conditions)
  {
    // Send experiment arm to right initial pose
    move_goal.desired_pose = initial_eef_pose;
    move_action_client_->sendGoal(move_goal);

    // Do a guarded approach in the -z direction of the table frame

    // Get ground truth of the contact point

    // Determine desired final pose

    // Initialize experiment.
  }
}

/*
  Returns the initial eef pose, base on vision or a pre-set value
*/
geometry_msgs::PoseStamped ManipulationClient::getInitialEefPose()
{
  geometry_msgs::PoseStamped initial_eef_pose;
  Eigen::Affine3d desired_initial_pose;

  // Define the intended orientation of the end-effector in the surface frame
  Eigen::AngleAxisd initial_orientation(initial_approach_angle_, Eigen::Vector3d::UnitX()); // need to check axis

  // Set the intended offset
  Eigen::Vector3d initial_offset;
  initial_offset << initial_pose_offset_[0], initial_pose_offset_[1], initial_pose_offset_[2];

  // Compose the transform
  desired_initial_pose.translate(initial_offset);
  desired_initial_pose.rotate(initial_orientation);

  // Convert to pose
  tf::poseEigenToMsg(desired_initial_pose, initial_eef_pose.pose);

  if (use_vision_)
  {
    initial_eef_pose.header.frame_id = surface_frame_name_;

    // Get in world frame
    listener_.transformPose(base_link_name_, initial_eef_pose, initial_eef_pose);

    initial_eef_pose.pose.position.x += surface_frame_pose_.pose.position.x;
    initial_eef_pose.pose.position.y += surface_frame_pose_.pose.position.y;
    initial_eef_pose.pose.position.z += surface_frame_pose_.pose.position.z;
  }
  else
  {
    // TODO: Get pre-set pose from config file
    initial_eef_pose.header.frame_id = base_link_name_;
    initial_eef_pose.pose.position.x = 0.804;
    initial_eef_pose.pose.position.y = 0.2;
    initial_eef_pose.pose.position.z = 0.1;
  }

  return initial_eef_pose;
}

/*
  Waits for the table frame to be available and saves the frame
  data.
*/
bool ManipulationClient::waitForTablePose(ros::Duration max_time)
{
  if (!listener_.waitForTransform(surface_frame_name_, base_link_name_, ros::Time(0), max_time))
  {
    return false;
  }

  surface_frame_pose_.header.stamp = ros::Time::now();
  surface_frame_pose_.header.frame_id = surface_frame_name_;
  surface_frame_pose_.pose.position.x = 0;
  surface_frame_pose_.pose.position.y = 0;
  surface_frame_pose_.pose.position.z = 0;
  surface_frame_pose_.pose.orientation.x = 0;
  surface_frame_pose_.pose.orientation.y = 0;
  surface_frame_pose_.pose.orientation.z = 0;
  surface_frame_pose_.pose.orientation.w = 1;

  listener_.transformPose(base_link_name_, surface_frame_pose_, surface_frame_pose_);
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_client");
  manipulation::ManipulationClient client;
  client.runExperiment();
  ros::spin();
  return 0;
}

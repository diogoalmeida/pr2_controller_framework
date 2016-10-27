#include <manipulation_client/manipulation_client.hpp>

namespace manipulation{

/*
  Get the client parameters and abort in case they are not found
*/
bool ManipulationClient::loadParams()
{
  if(!nh_.getParam("vision/surface_frame_name", surface_frame_name_))
  {
    ROS_ERROR("No surface frame name defined (vision/surface_frame_name_)");
    return false;
  }

  if(!nh_.getParam("base_link_name", base_link_name_))
  {
    ROS_ERROR("No base link frame name defined (base_link_name)");
    return false;
  }

  if(!nh_.getParam("initialization/initial_pose_offset", initial_pose_offset_))
  {
    ROS_ERROR("No inital eef offset defined (initialization/initial_pose_offset)");
    return false;
  }
}

/*
  Initializes the robot pose based on tag information. Commands the initial
  approach (another action? here?). Monitors vision information to compute
  ground truth data, and registers feedback.
*/
void ManipulationClient::runExperiment()
{
  ROS_INFO("%s client waiting for server", action_name_.c_str());
  if(!action_client_.waitForServer(ros::Duration(server_timeout_)))
  {
    ROS_ERROR("%s was not found. Aborting", action_name_.c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("%s client waiting for table pose", action_name_.c_str());
  if(!waitForTablePose(ros::Duration(vision_timeout_)))
  {
    ROS_ERROR("Could not find table frame. Aborting");
    ros::shutdown();
    return;
  }

  // At this point I have knowledge of the arm that I want to move (tool frame)
  // and what is my desired initial pose for the robotic manipulator

  //while(experiment_conditions)
  {
    // Send experiment arm to right initial pose

    // Do a guarded approach in the -z direction of the table frame

    // Get ground truth of the contact point

    // Determine desired final pose

    // Initialize experiment.
  }
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

  initial_eef_pose_ = surface_frame_pose_;
  initial_eef_pose_.pose.position.x = initial_pose_offset_[0];
  initial_eef_pose_.pose.position.y = initial_pose_offset_[1];
  initial_eef_pose_.pose.position.z = initial_pose_offset_[2];
  initial_eef_pose_.pose.orientation.x = 1;
  initial_eef_pose_.pose.orientation.w = 0;

  listener_.transformPose(base_link_name_, surface_frame_pose_, surface_frame_pose_);
  return true;
}

}

int main(int argc, char **argv)
{
  return 0;
}

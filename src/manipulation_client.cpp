#include <manipulation_client/manipulation_client.hpp>

namespace manipulation{

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

}

}

int main(int argc, char **argv)
{
  return 0;
}

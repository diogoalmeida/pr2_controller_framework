#include <manipulation_client/manipulation_client.hpp>

namespace manipulation{

ManipulationClient::ManipulationClient()
{
  nh_ = ros::NodeHandle("~");
  if(!loadParams())
  {
    ros::shutdown();
    return;
  }

  gravity_compensation_client_ = nh_.serviceClient<std_srvs::Empty>(gravity_compensation_service_name_);
  load_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::LoadController>("/pr2_controller_manager/load_controller");
  unload_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::UnloadController>("/pr2_controller_manager/unload_controller");
  switch_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");

  if (!loadControllers())
  {
    unloadControllers();
    ROS_ERROR("Failed to load the PR2 controllers required for the experiment");
    ros::shutdown();
    return;
  }

  action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_clients::ManipulationAction>(nh_, cartesian_client_action_name_, false);
  action_server_->registerGoalCallback(boost::bind(&ManipulationClient::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&ManipulationClient::preemptCB, this));
  manipulation_action_client_ = nullptr;
  approach_action_client_ = nullptr;
  move_action_client_ = nullptr;
  feedback_thread_ = boost::thread(&ManipulationClient::publishFeedback, this);
}

ManipulationClient::~ManipulationClient()
{
  action_server_->shutdown();

  unloadControllers();
  destroyActionClients();

  if (feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }
}

/*
  Makes sure that the required system controllers are loaded in the PR2
*/
bool ManipulationClient::loadControllers()
{
  if (!loadController(move_controller_name_))
  {
    return false;
  }
  if (!loadController(manipulation_controller_name_))
  {
    return false;
  }
  if (!loadController(approach_controller_name_))
  {
    return false;
  }

  return true;
}

/*
  Loads the given controller in the PR2
*/
bool ManipulationClient::loadController(std::string controller_name)
{
  pr2_mechanism_msgs::LoadController load_srv;

  load_srv.request.name = controller_name;
  if(!load_controllers_client_.call(load_srv))
  {
    ROS_ERROR("Error calling the load controller server!");
    return false;
  }
  else
  {
    if(load_srv.response.ok)
    {
      ROS_INFO("Successfully loaded controller %s", controller_name.c_str());
      return true;
    }
    else
    {
      ROS_ERROR("Failed in loading the controller %s!", controller_name.c_str());
      return false;
    }
  }
}

/*
  Makes sure that the required system controllers are unloaded from the PR2
*/
bool ManipulationClient::unloadControllers()
{
  if (!unloadController(move_controller_name_))
  {
    return false;
  }
  if (!unloadController(manipulation_controller_name_))
  {
    return false;
  }
  if (!unloadController(approach_controller_name_))
  {
    return false;
  }

  return true;
}

/*
  Unloads the given controller from the PR2
*/
bool ManipulationClient::unloadController(std::string controller_name)
{
  pr2_mechanism_msgs::UnloadController unload_srv;

  unload_srv.request.name = controller_name;
  if(!unload_controllers_client_.call(unload_srv))
  {
    ROS_ERROR("Error calling the unload controller server!");
    return false;
  }
  else
  {
    if(unload_srv.response.ok)
    {
      ROS_INFO("Successfully unloaded controller %s", controller_name.c_str());
      return true;
    }
    else
    {
      ROS_ERROR("Failed in unloading the controller %s!", controller_name.c_str());
      return false;
    }
  }
}


/*
  Starts the given controller, and stops all the other controllers relevant to the
  experiment
*/
bool ManipulationClient::switchToController(std::string controller_name)
{
  pr2_mechanism_msgs::SwitchController switch_srv;

  switch_srv.request.start_controllers.push_back(controller_name);
  switch_srv.request.stop_controllers.push_back("r_arm_controller");
  switch_srv.request.stop_controllers.push_back("l_arm_controller");

  if (controller_name != move_controller_name_)
  {
    switch_srv.request.stop_controllers.push_back(move_controller_name_);
  }

  if (controller_name != approach_controller_name_)
  {
    switch_srv.request.stop_controllers.push_back(approach_controller_name_);
  }

  if (controller_name != manipulation_controller_name_)
  {
    switch_srv.request.stop_controllers.push_back(manipulation_controller_name_);
  }

  if(!switch_controllers_client_.call(switch_srv))
  {
    ROS_ERROR("Error calling the switch controller server!");
    return false;
  }
  else
  {
    if(switch_srv.response.ok)
    {
      ROS_INFO("Successfully started controller %s", controller_name.c_str());
      return true;
    }
    else
    {
      ROS_ERROR("Failed in starting the controller %s!", controller_name.c_str());
      return false;
    }
  }
}

/*
  Makes sure the action clients cancel the goals and are properly deleted
*/
void ManipulationClient::destroyActionClients()
{
  if (manipulation_action_client_)
  {
    manipulation_action_client_->cancelAllGoals();
    delete manipulation_action_client_;
    manipulation_action_client_ = nullptr;
  }

  if (approach_action_client_)
  {
    approach_action_client_->cancelAllGoals();
    delete approach_action_client_;
    approach_action_client_ = nullptr;
  }

  if (move_action_client_)
  {
    move_action_client_->cancelAllGoals();
    delete move_action_client_;
    move_action_client_ = nullptr;
  }
}

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

  if(!nh_.getParam("initialization/actionlib_server_names/manipulation_action_name", manipulation_action_name_))
  {
    ROS_ERROR("No manipulation action name defined (initialization/actionlib_server_names/manipulation_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/approach_action_name", approach_action_name_))
  {
    ROS_ERROR("No approach action name defined (initialization/actionlib_server_names/approach_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/move_action_name", move_action_name_))
  {
    ROS_ERROR("No move action name defined (initialization/actionlib_server_names/move_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_names/manipulation_controller", manipulation_controller_name_))
  {
    ROS_ERROR("No manipulation action name defined (initialization/controller_names/manipulation_controller)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_names/approach_controller", approach_controller_name_))
  {
    ROS_ERROR("No approach action name defined (initialization/controller_names/approach_controller)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_names/move_controller", move_controller_name_))
  {
    ROS_ERROR("No move action name defined (initialization/controller_names/move_controller)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/client_action_name", cartesian_client_action_name_))
  {
    ROS_ERROR("No cartesian client action name defined (initialization/actionlib_server_names/client_action_name)");
    return false;
  }

  if(!nh_.getParam("experiment/feedback_rate", feedback_hz_))
  {
    ROS_ERROR("No feedback frequency defined (initialization/feedback_rate)");
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

  if(!nh_.getParam("initialization/server_timeout", server_timeout_))
  {
    ROS_ERROR("No server timeout defined (initialization/server_timeout)");
    return false;
  }

  return true;
}

/*
  Periodically publish feedback reporting the action being run
*/
void ManipulationClient::publishFeedback()
{
  try
  {
    while(ros::ok())
    {
      if (action_server_->isActive())
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        feedback_.current_action = current_action_;
        action_server_->publishFeedback(feedback_);
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
    }
  }
  catch(const boost::thread_interrupted &)
  {
    return;
  }
}

/*
  Upon preemption, cancel all goals and destroy the action clients
*/
void ManipulationClient::preemptCB()
{
  ROS_WARN("The manipulation client was preempted! Preempting action servers...");
  destroyActionClients();
  current_action_.clear();
}

/*
  When receiving a new goal, initialize the action clients
  and get vision information
*/
void ManipulationClient::goalCB()
{
  action_server_->acceptNewGoal();

  if (!action_server_->isPreemptRequested())
  {
    ROS_INFO("%s received a new goal!", cartesian_client_action_name_.c_str());
    manipulation_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction>(manipulation_action_name_, true);
    approach_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction>(approach_action_name_, true);
    move_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction>(move_action_name_, true);

    ROS_INFO("%s client waiting for server", manipulation_action_name_.c_str());
    if(!manipulation_action_client_->waitForServer(ros::Duration(server_timeout_)))
    {
      ROS_ERROR("%s was not found. Aborting", manipulation_action_name_.c_str());
      action_server_->setAborted();
      destroyActionClients();
      return;
    }

    ROS_INFO("%s client waiting for server", approach_action_name_.c_str());
    if(!approach_action_client_->waitForServer(ros::Duration(server_timeout_)))
    {
      ROS_ERROR("%s was not found. Aborting", approach_action_name_.c_str());
      action_server_->setAborted();
      destroyActionClients();
      return;
    }

    ROS_INFO("%s client waiting for server", move_action_name_.c_str());
    if(!move_action_client_->waitForServer(ros::Duration(server_timeout_)))
    {
      ROS_ERROR("%s was not found. Aborting", move_action_name_.c_str());
      action_server_->setAborted();
      destroyActionClients();
      return;
    }

    if(use_vision_)
    {
      ROS_INFO("Cartesian client waiting for table pose");
      if(!waitForTablePose(ros::Duration(vision_timeout_)))
      {
        ROS_ERROR("Could not find table frame. Aborting");
        action_server_->setAborted();
        destroyActionClients();
        return;
      }
    }

    if(!sim_mode_)
    {
      // Zero the ft sensor readings
      std_srvs::Empty srv;

      if(!gravity_compensation_client_.call(srv))
      {
        ROS_ERROR("Error calling the gravity compensation server!");
        action_server_->setAborted();
        destroyActionClients();
        return;
      }
    }
  }
  else
  {
    ROS_WARN("Received a new goal, but a pending preempt was requested. Will ignore");
    action_server_->setPreempted();
  }
}

/*
  Initializes the robot pose based on tag information. Commands the initial
  approach (another action? here?). Monitors vision information to compute
  ground truth data, and registers feedback.
*/
void ManipulationClient::runExperiment()
{
  action_server_->start();
  ROS_INFO("Started the manipulation client action server: %s", cartesian_client_action_name_.c_str());
  while (ros::ok())
  {
    if (action_server_->isActive())
    {
      // At this point I have knowledge of the arm that I want to move (tool frame)
      // and I can compute the desired initial pose of the end-effector
      geometry_msgs::PoseStamped initial_eef_pose;

      if(!getInitialEefPose(initial_eef_pose))
      {
        ROS_ERROR("Failed to get surface frame pose!");
        destroyActionClients();
        action_server_->setAborted();
        continue;
      }

      pr2_cartesian_controllers::MoveGoal move_goal;
      pr2_cartesian_controllers::GuardedApproachGoal approach_goal;
      pr2_cartesian_controllers::ManipulationControllerGoal manipulation_goal;

      ROS_INFO("Starting experiment!");
      //while(experiment_conditions)
      {
        // Send experiment arm to right initial pose
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = move_action_name_;
        }
        move_goal.desired_pose = initial_eef_pose;

        switchToController(move_controller_name_);
        move_action_client_->sendGoal(move_goal);
        move_action_client_->waitForResult();

        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = approach_action_name_;
        }
        // Do a guarded approach in the -z direction of the table frame

        // Get ground truth of the contact point

        // Determine desired final pose

        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = manipulation_action_name_;
        }
        // Initialize experiment.
      }
    }
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
  }
}

/*
  Gets the initial eef pose, based on vision or a pre-set value.
  Returns false if it takes more than vision_timeout_ seconds to obtain a valid
  surface frame transform
*/
bool ManipulationClient::getInitialEefPose(geometry_msgs::PoseStamped & pose)
{
  geometry_msgs::PoseStamped initial_eef_pose;
  Eigen::Affine3d desired_initial_pose = Eigen::Affine3d::Identity();

  ROS_INFO("Getting initial orientation");
  // Define the intended orientation of the end-effector in the surface frame
  Eigen::AngleAxisd initial_orientation(initial_approach_angle_, Eigen::Vector3d::UnitZ()); // need to check axis

  // Set the intended offset
  Eigen::Vector3d initial_offset;
  initial_offset << initial_pose_offset_[0], initial_pose_offset_[1], initial_pose_offset_[2];

  ROS_INFO("Initial pose offset: ");
  std::cout << initial_offset << std::endl;

  // Compose the transform
  desired_initial_pose.translate(initial_offset);
  desired_initial_pose.prerotate(initial_orientation);

  ROS_INFO("Transform:");
  std::cout << desired_initial_pose.translation() << std::endl;
  std::cout << desired_initial_pose.rotation();

  ROS_INFO("Converting to message");
  // Convert to pose
  tf::poseEigenToMsg(desired_initial_pose, initial_eef_pose.pose);

  if (use_vision_)
  {
    initial_eef_pose.header.frame_id = surface_frame_name_;
    initial_eef_pose.header.stamp = ros::Time(0);

    ROS_INFO("Transforming to the world frame");
    // Get in world frame
    listener_.transformPose(base_link_name_, initial_eef_pose, initial_eef_pose);

    // initial_eef_pose.pose.position.x += surface_frame_pose_.pose.position.x;
    // initial_eef_pose.pose.position.y += surface_frame_pose_.pose.position.y;
    // initial_eef_pose.pose.position.z += surface_frame_pose_.pose.position.z;
  }
  else
  {
    // TODO: Get pre-set pose from config file
    initial_eef_pose.header.frame_id = base_link_name_;
    initial_eef_pose.pose.position.x = 0.804;
    initial_eef_pose.pose.position.y = 0.2;
    initial_eef_pose.pose.position.z = 0.1;
  }

  ROS_INFO("Going to:");
  std::cout << initial_eef_pose.pose.position.x << " " << initial_eef_pose.pose.position.y << " " << initial_eef_pose.pose.position.z << " " << initial_eef_pose.pose.orientation.x << " " << initial_eef_pose.pose.orientation.y << " " << initial_eef_pose.pose.orientation.z << " " << initial_eef_pose.pose.orientation.w << std::endl;

  pose = initial_eef_pose;
  return true;
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

  surface_frame_pose_.header.stamp = ros::Time(0);
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
  return 0;
}

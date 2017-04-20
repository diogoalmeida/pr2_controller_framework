#include <manipulation_client/manipulation_client.hpp>

using namespace pr2_cartesian_clients;
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
  logging_service_client_ = nh_.serviceClient<pr2_cartesian_clients::LogMessages>(logging_service_);
  manipulation_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::ManipulationControllerAction>(manipulation_action_name_, true);
  approach_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction>(approach_action_name_, true);
  move_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction>(move_action_name_, true);
  action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_clients::ManipulationAction>(nh_, cartesian_client_action_name_, false);
  action_server_->registerGoalCallback(boost::bind(&ManipulationClient::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&ManipulationClient::preemptCB, this));
  feedback_thread_ = boost::thread(&ManipulationClient::publishFeedback, this);
}

ManipulationClient::~ManipulationClient()
{
  action_server_->shutdown();
  destroyActionClients();

  if (feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }
}

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

bool ManipulationClient::loadParams()
{
  if(!nh_.getParam("experiment/arm", arm_))
  {
    ROS_ERROR("No arm chosen (experiment/arm)");
    return false;
  }

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

  if(!nh_.getParam("initialization/controller_timeouts/move_timeout", move_action_time_limit_))
  {
    ROS_ERROR("No move timeout defined (initialization/controller_timeouts/move_timeout)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_timeouts/approach_timeout", approach_action_time_limit_))
  {
    ROS_ERROR("No move timeout defined (initialization/controller_timeouts/approach_timeout)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_timeouts/manipulation_timeout", manipulation_action_time_limit_))
  {
    ROS_ERROR("No move timeout defined (initialization/controller_timeouts/manipulation_timeout)");
    return false;
  }

  if(!nh_.getParam("experiment/feedback_rate", feedback_hz_))
  {
    ROS_ERROR("No feedback frequency defined (initialization/feedback_rate)");
    return false;
  }

  if(!nh_.getParam("experiment/bag_prefix", bag_prefix_))
  {
    ROS_ERROR("Need to set bag_prefix (experiment/bag_prefix)");
    return false;
  }

  if(!nh_.getParam("experiment/num_of_experiments", num_of_experiments_))
  {
    ROS_ERROR("No number of experiments defined (experiment/num_of_experiments)");
    return false;
  }

  if(!nh_.getParam("experiment/logging/toggle_logging_service", logging_service_))
  {
    ROS_ERROR("No logging toggle service provided (experiment/logging/toggle_logging_service)");
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

  if(!nh_.getParam("experiment/approach_velocity", approach_velocity_))
  {
    ROS_ERROR("Need to set approach_velocity (experiment/approach_velocity)");
    return false;
  }

  if(!nh_.getParam("experiment/approach_force", approach_force_))
  {
    ROS_ERROR("Need to set approach_force (experiment/approach_force)");
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
  if(!nh_.getParam("experiment/goal_theta", goal_theta_))
  {
    ROS_ERROR("Need to set goal_theta (experiment/goal_theta)");
    return false;
  }

  if(!nh_.getParam("experiment/goal_x", goal_x_))
  {
    ROS_ERROR("Need to set goal_x (experiment/goal_x)");
    return false;
  }

  if(!nh_.getParam("experiment/goal_force", goal_force_))
  {
    ROS_ERROR("Need to set goal_force (experiment/goal_force)");
    return false;
  }

  std::vector<std::string> exclusion_list;
  if (nh_.getParam("initialization/exclude_controller_names", exclusion_list))
  {
    for (int i = 0; i < exclusion_list.size(); i++)
    {
      ROS_INFO("Adding exclusion: %s", exclusion_list[i].c_str());
      controller_runner_.addException(exclusion_list[i]);
    }
  }

  return true;
}

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
        feedback_.progress = std::string("Performed ") + std::to_string(current_iter_) + std::string(" out of ") + std::to_string(num_of_experiments_);
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

void ManipulationClient::preemptCB()
{
  ROS_WARN("The manipulation client was preempted!");
  action_server_->setPreempted();
  controller_runner_.unloadAll();
  current_action_.clear();
}

void ManipulationClient::goalCB()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  boost::shared_ptr<const pr2_cartesian_clients::ManipulationGoal> goal = action_server_->acceptNewGoal();

  if (!action_server_->isPreemptRequested())
  {
    ROS_INFO("%s received a new goal!", cartesian_client_action_name_.c_str());

    if(use_vision_)
    {
      ROS_INFO("Cartesian client waiting for table pose");
      if(!waitForTablePose(ros::Duration(vision_timeout_)))
      {
        ROS_ERROR("Could not find table frame. Aborting");
        action_server_->setAborted();
        return;
      }
    }

    if (goal->use_goal)
    {
      ROS_INFO("Using goal");
      initial_pose_offset_.push_back(goal->initial_pose_offset.x);
      initial_pose_offset_.push_back(goal->initial_pose_offset.y);
      initial_pose_offset_.push_back(goal->initial_pose_offset.z);
      initial_approach_angle_ = goal->initial_approach_angle;
      manipulation_action_time_limit_ = goal->manipulation_timeout;
      num_of_experiments_ = goal->num_of_experiments;
      bag_prefix_ = goal->bag_prefix.data;
      goal_x_ = goal->desired_state.x;
      goal_theta_ = goal->desired_state.y;
      goal_force_ = goal->desired_state.z;
      arm_ = goal->arm;

      if (goal->randomize_desired_state)
      {
        noise_x_d_ = std::uniform_real_distribution<double>(goal->x_min, goal->x_max);
        noise_theta_d_ = std::uniform_real_distribution<double>(goal->theta_min, goal->theta_max);
        noise_f_d_ = std::uniform_real_distribution<double>(goal->f_min, goal->f_max);
      }
      else
      {
        noise_x_d_ = std::uniform_real_distribution<double>(0, 0);
        noise_theta_d_ = std::uniform_real_distribution<double>(0, 0);
        noise_f_d_ = std::uniform_real_distribution<double>(0, 0);
      }
    }
    else
    {
      noise_x_d_ = std::uniform_real_distribution<double>(0, 0);
      noise_theta_d_ = std::uniform_real_distribution<double>(0, 0);
      noise_f_d_ = std::uniform_real_distribution<double>(0, 0);
    }
  }
  else
  {
    ROS_WARN("Received a new goal, but a pending preempt was requested. Will ignore");
    action_server_->setPreempted();
  }
}

void ManipulationClient::runExperiment()
{
  geometry_msgs::PoseStamped initial_eef_pose;
  ros::Time init, curr;
  bool got_eef_pose = false;
  int current_iter = 1;

  action_server_->start();
  ROS_INFO("Started the manipulation client action server: %s", cartesian_client_action_name_.c_str());

  while (ros::ok())
  {
    if (action_server_->isActive())
    {
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_); // to wait if goal is being processed
      }
      controller_runner_.unloadAll();
      // At this point I have knowledge of the arm that I want to move (tool frame)
      // and I can compute the desired initial pose of the end-effector
      if (!got_eef_pose)
      {
        if(!getInitialEefPose(initial_eef_pose))
        {
          ROS_ERROR("Failed to get surface frame pose!");
          action_server_->setAborted();
          continue;
        }
        got_eef_pose = true;
      }

      pr2_cartesian_controllers::MoveGoal move_goal;
      pr2_cartesian_controllers::GuardedApproachGoal approach_goal;
      pr2_cartesian_controllers::ManipulationControllerGoal manipulation_goal;

      ROS_INFO("Starting experiment!");
      current_iter = 1;

      while(action_server_->isActive() && current_iter <= num_of_experiments_)
      {
        controller_runner_.unloadAll();
        // Send experiment arm to right initial pose
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = move_action_name_;
        }
        move_goal.arm = arm_;
        move_goal.desired_pose = initial_eef_pose;

        if (!controller_runner_.runController(move_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", move_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        bool move_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::MoveAction,
                              pr2_cartesian_controllers::MoveGoal,
                              pr2_cartesian_clients::ManipulationAction>
                                (move_action_client_, move_goal, action_server_, server_timeout_, move_action_time_limit_, move_timeout))
        {
          ROS_ERROR("Error in the move action. Aborting.");
          action_server_->setAborted();
          continue;
        }
        ROS_INFO("Move action succeeded!");

        if(!sim_mode_)
        {
          // Zero the ft sensor readings
          std_srvs::Empty srv;

          if(!gravity_compensation_client_.call(srv))
          {
            ROS_ERROR("Error calling the gravity compensation server!");
            action_server_->setAborted();
            return;
          }
        }

        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = approach_action_name_;
        }

        if (!controller_runner_.runController(approach_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", approach_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        approach_goal.arm = arm_;
        approach_goal.approach_command.twist.linear.z = approach_velocity_;
        approach_goal.contact_force = approach_force_;

        bool approach_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::GuardedApproachAction,
                              pr2_cartesian_controllers::GuardedApproachGoal,
                              pr2_cartesian_clients::ManipulationAction>
                                (approach_action_client_, approach_goal, action_server_, server_timeout_, approach_action_time_limit_, approach_timeout))
        {
          ROS_ERROR("Error in the approach action. Aborting.");
          action_server_->setAborted();
          continue;
        }

        ROS_INFO("Approach action succeeded!");
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = manipulation_action_name_;
          current_iter_ = current_iter;
        }


        if (!controller_runner_.runController(manipulation_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", manipulation_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        std::string bag_name;
        bag_name = std::string(bag_prefix_) + std::string("_") + std::to_string(current_iter);
        ROS_INFO("Running experiment %d/%d", current_iter, num_of_experiments_);
        {
          pr2_cartesian_clients::LogMessages srv;
          srv.request.log_type = srv.request.START_LOGGING;
          srv.request.name = bag_name;
          srv.request.max_record_time = manipulation_action_time_limit_ + 10;
          if (!logging_service_client_.call(srv))
          {
            ROS_WARN("Error calling the logging service, will not be able to log experiment");
          }
        }

        manipulation_goal.surface_frame = surface_frame_pose_;

        Eigen::Affine3d surface_pose_eigen;
        Eigen::Vector3d rotation_axis = -Eigen::Vector3d::UnitY();

        tf::poseMsgToEigen(surface_frame_pose_.pose, surface_pose_eigen);

        surface_pose_eigen = surface_pose_eigen*Eigen::AngleAxisd(0.5, rotation_axis);

        manipulation_goal.arm = arm_;
        manipulation_goal.x_d = goal_x_ + noise_x_d_(noise_generator_);
        manipulation_goal.theta_d = goal_theta_ + noise_theta_d_(noise_generator_);
        manipulation_goal.desired_contact_force = goal_force_ + noise_f_d_(noise_generator_);
        manipulation_goal.is_debug = false;
        manipulation_goal.use_debug_eef_to_grasp = false;
        manipulation_goal.use_surface_rotation_axis = true;

        bool manipulation_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::ManipulationControllerAction,
                              pr2_cartesian_controllers::ManipulationControllerGoal,
                              pr2_cartesian_clients::ManipulationAction>
                                (manipulation_action_client_, manipulation_goal, action_server_, server_timeout_, manipulation_action_time_limit_, manipulation_timeout))
        {
          if (!manipulation_timeout)
          {
            ROS_ERROR("Error in the manipulation action.");
            {
              pr2_cartesian_clients::LogMessages srv;
              srv.request.log_type = srv.request.DISCARD_BAG;
              if (!logging_service_client_.call(srv))
              {
                ROS_WARN("Error calling the logging service, will not be able to log experiment");
              }
            }
            continue;
          }
        }

        {
          pr2_cartesian_clients::LogMessages srv;
          srv.request.log_type = srv.request.SAVE_BAG;
          if (!logging_service_client_.call(srv))
          {
            ROS_WARN("Error calling the logging service, will not be able to log experiment");
          }
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        current_iter++;
        ros::spinOnce();
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
      }

      ROS_INFO("Experiment done!");
      controller_runner_.unloadAll();
      action_server_->setSucceeded();
    }
    else
    {
      got_eef_pose = false;
    }

    current_iter = 1;
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
  }
}

bool ManipulationClient::getInitialEefPose(geometry_msgs::PoseStamped & pose)
{
  geometry_msgs::PoseStamped initial_eef_pose;
  Eigen::Affine3d desired_initial_pose = Eigen::Affine3d::Identity();

  ROS_INFO("Getting initial orientation");
  // Define the intended orientation of the end-effector in the surface frame
  Eigen::AngleAxisd initial_orientation(initial_approach_angle_, Eigen::Vector3d::UnitY()); // need to check axis

  // Set the intended offset
  Eigen::Vector3d initial_offset;
  initial_offset << initial_pose_offset_[0], initial_pose_offset_[1], initial_pose_offset_[2];

  ROS_INFO("Initial pose offset: ");
  std::cout << initial_offset << std::endl;

  // Compose the transform
  desired_initial_pose = Eigen::Translation3d(initial_offset) * initial_orientation;
  // desired_initial_pose.rotate(initial_orientation);
  // desired_initial_pose.translate(initial_offset);

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

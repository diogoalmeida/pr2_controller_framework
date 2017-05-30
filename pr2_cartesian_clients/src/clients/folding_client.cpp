#include <clients/folding_client.hpp>

using namespace pr2_cartesian_clients;
namespace manipulation{

FoldingClient::FoldingClient()
{
  nh_ = ros::NodeHandle("~");
  if(!loadParams())
  {
    ros::shutdown();
    return;
  }

  gravity_compensation_client_ = nh_.serviceClient<std_srvs::Empty>(gravity_compensation_service_name_);
  logging_service_client_ = nh_.serviceClient<pr2_cartesian_clients::LogMessages>(logging_service_);
  folding_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::FoldingControllerAction>(folding_action_name_, true);
  approach_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::GuardedApproachAction>(approach_action_name_, true);
  move_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction>(move_action_name_, true);
  action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_clients::FoldingAction>(nh_, cartesian_client_action_name_, false);
  action_server_->registerGoalCallback(boost::bind(&FoldingClient::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&FoldingClient::preemptCB, this));
  feedback_thread_ = boost::thread(&FoldingClient::publishFeedback, this);
}

FoldingClient::~FoldingClient()
{
  action_server_->shutdown();
  destroyActionClients();

  if (feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }
}

void FoldingClient::destroyActionClients()
{
  if (folding_action_client_)
  {
    folding_action_client_->cancelAllGoals();
    delete folding_action_client_;
    folding_action_client_ = nullptr;
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

bool FoldingClient::loadParams()
{
  if(!nh_.getParam("experiment/rod_arm", rod_arm_))
  {
    ROS_ERROR("No rod arm chosen (experiment/rod_arm)");
    return false;
  }

  if(!nh_.getParam("experiment/surface_arm", surface_arm_))
  {
    ROS_ERROR("No surface arm chosen (experiment/surface_arm)");
    return false;
  }

  if(!nh_.getParam("experiment/base_link_name", base_link_name_))
  {
    ROS_ERROR("No base link frame name defined (base_link_name)");
    return false;
  }

  if(!nh_.getParam("initialization/initial_approach_angle", initial_approach_angle_))
  {
    ROS_ERROR("No inital approach angle defined (initialization/initial_approach_angle)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/folding_action_name", folding_action_name_))
  {
    ROS_ERROR("No folding action name defined (initialization/actionlib_server_names/folding_action_name)");
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

  if(!nh_.getParam("initialization/controller_names/folding_controller", folding_controller_name_))
  {
    ROS_ERROR("No folding action name defined (initialization/controller_names/folding_controller)");
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
    ROS_ERROR("No approach timeout defined (initialization/controller_timeouts/approach_timeout)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_timeouts/folding_timeout", folding_action_time_limit_))
  {
    ROS_ERROR("No folding timeout defined (initialization/controller_timeouts/folding_timeout)");
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

  if(!getPose("experiment/initial_pose/rod", initial_rod_pose_))
  {
    ROS_ERROR("Failed to get initial rod pose (experiment/initial_pose/rod)");
    return false;
  }

  if(!getPose("experiment/initial_pose/surface", initial_surface_pose_))
  {
    ROS_ERROR("Failed to get initial surface pose (experiment/initial_pose/surface)");
    return false;
  }

  if(!nh_.getParam("experiment/surface_arm_tool_frame", surface_frame_name_))
  {
    ROS_ERROR("Failed to get initial surface_arm_tool_frame (experiment/surface_arm_tool_frame)");
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

void FoldingClient::publishFeedback()
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

void FoldingClient::preemptCB()
{
  ROS_WARN("The folding client was preempted!");
  action_server_->setPreempted();
  controller_runner_.unloadAll();
  current_action_.clear();
}

void FoldingClient::goalCB()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  boost::shared_ptr<const pr2_cartesian_clients::FoldingGoal> goal = action_server_->acceptNewGoal();

  if (!action_server_->isPreemptRequested())
  {
    ROS_INFO("%s received a new goal!", cartesian_client_action_name_.c_str());

    if (goal->use_goal)
    {
      ROS_INFO("Using goal");
      initial_approach_angle_ = goal->initial_approach_angle;
      folding_action_time_limit_ = goal->folding_timeout;
      num_of_experiments_ = goal->num_of_experiments;
      bag_prefix_ = goal->bag_prefix.data;
      goal_x_ = goal->desired_state.x;
      goal_theta_ = goal->desired_state.y;
      goal_force_ = goal->desired_state.z;
      rod_arm_ = goal->rod_arm;
      surface_arm_ = goal->surface_arm;

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

void FoldingClient::runExperiment()
{
  ros::Time init, curr;
  bool got_eef_pose = false;
  int current_iter = 1;

  action_server_->start();
  ROS_INFO("Started the folding client action server: %s", cartesian_client_action_name_.c_str());

  while (ros::ok())
  {
    if (action_server_->isActive())
    {
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_); // to wait if goal is being processed
      }

      pr2_cartesian_controllers::MoveGoal move_goal;
      pr2_cartesian_controllers::GuardedApproachGoal approach_goal;
      pr2_cartesian_controllers::FoldingControllerGoal folding_goal;

      ROS_INFO("Starting experiment!");
      current_iter = 1;

      while(action_server_->isActive() && current_iter <= num_of_experiments_)
      {
        // controller_runner_.unloadAll();
        // Send rod arm to right initial pose
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = move_action_name_ + std::string(" (rod arm)");
        }

        move_goal.arm = rod_arm_;
        move_goal.desired_pose = initial_rod_pose_;

        if (!controller_runner_.runController(move_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", move_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        controller_runner_.stopController("l_arm_controller");
        controller_runner_.stopController("r_arm_controller");

        bool move_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::MoveAction,
                              pr2_cartesian_controllers::MoveGoal,
                              pr2_cartesian_clients::FoldingAction>
                                (move_action_client_, move_goal, action_server_, server_timeout_, move_action_time_limit_, move_timeout))
        {
          ROS_ERROR("Error in the move action. Aborting.");
          action_server_->setAborted();
          continue;
        }
        ROS_INFO("Move action succeeded!");

        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = move_action_name_ + std::string(" (surface arm)");
        }

        // Send the surface arm to the initial pose
        move_goal.arm = surface_arm_;
        move_goal.desired_pose = initial_surface_pose_;

        move_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::MoveAction,
                              pr2_cartesian_controllers::MoveGoal,
                              pr2_cartesian_clients::FoldingAction>
                                (move_action_client_, move_goal, action_server_, server_timeout_, move_action_time_limit_, move_timeout))
        {
          ROS_ERROR("Error in the move action. Aborting.");
          action_server_->setAborted();
          continue;
        }
        ROS_INFO("Move action succeeded!");

        // Zero the ft sensor readings
        std_srvs::Empty srv;

        if(!gravity_compensation_client_.call(srv))
        {
          ROS_ERROR("Error calling the gravity compensation server!");
          action_server_->setAborted();
          return;
        }

        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = approach_action_name_;
        }

        controller_runner_.runController("l_arm_controller");
        controller_runner_.runController("r_arm_controller");

        if (!controller_runner_.runController(approach_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", approach_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        controller_runner_.stopController("l_arm_controller");
        controller_runner_.stopController("r_arm_controller");

        approach_goal.arm = rod_arm_;
        approach_goal.approach_command.header.frame_id = surface_frame_name_;
        approach_goal.approach_command.header.stamp = ros::Time(0);
        approach_goal.approach_command.twist.linear.z = approach_velocity_; // TODO: Needs to be a surface frame vector in the base frame
        approach_goal.contact_force = approach_force_;

        bool approach_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::GuardedApproachAction,
                              pr2_cartesian_controllers::GuardedApproachGoal,
                              pr2_cartesian_clients::FoldingAction>
                                (approach_action_client_, approach_goal, action_server_, server_timeout_, approach_action_time_limit_, approach_timeout))
        {
          ROS_ERROR("Error in the approach action. Aborting.");
          action_server_->setAborted();
          continue;
        }

        ROS_INFO("Approach action succeeded!");
        {
          boost::lock_guard<boost::mutex> guard(reference_mutex_);
          current_action_ = folding_action_name_;
          current_iter_ = current_iter;
        }

        controller_runner_.runController("l_arm_controller");
        controller_runner_.runController("r_arm_controller");

        if (!controller_runner_.runController(folding_controller_name_))
        {
          ROS_ERROR("Failed to run the controller %s", folding_action_name_.c_str());
          action_server_->setAborted();
          continue;
        }

        controller_runner_.stopController("l_arm_controller");
        controller_runner_.stopController("r_arm_controller");

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        std::string bag_name;
        bag_name = std::string(bag_prefix_) + std::string("_") + std::to_string(current_iter);
        ROS_INFO("Running experiment %d/%d", current_iter, num_of_experiments_);
        {
          pr2_cartesian_clients::LogMessages srv;
          srv.request.log_type = srv.request.START_LOGGING;
          srv.request.name = bag_name;
          srv.request.max_record_time = folding_action_time_limit_ + 10;
          if (!logging_service_client_.call(srv))
          {
            ROS_WARN("Error calling the logging service, will not be able to log experiment");
          }
        }

        folding_goal.rod_arm = rod_arm_;
        folding_goal.surface_arm = surface_arm_;
        folding_goal.position_offset = goal_x_ + noise_x_d_(noise_generator_);
        folding_goal.orientation_goal = goal_theta_ + noise_theta_d_(noise_generator_);
        folding_goal.force_goal = goal_force_ + noise_f_d_(noise_generator_);

        bool folding_timeout = false;
        if (!monitorActionGoal<pr2_cartesian_controllers::FoldingControllerAction,
                              pr2_cartesian_controllers::FoldingControllerGoal,
                              pr2_cartesian_clients::FoldingAction>
                                (folding_action_client_, folding_goal, action_server_, server_timeout_, folding_action_time_limit_, folding_timeout))
        {
          if (!folding_timeout)
          {
            ROS_ERROR("Error in the folding action.");
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

bool FoldingClient::getPose(const std::string &param, geometry_msgs::PoseStamped &pose)
{
  std::vector<double> pose_std;

  if(!nh_.getParam(param, pose_std))
  {
    ROS_ERROR("Missing pose parameter (%s)", param.c_str());
    return false;
  }

  if (pose_std.size() != 7)
  {
    ROS_ERROR("Expected pose parameter with dimension 7, got %lu", pose_std.size());
    return false;
  }

  pose.pose.position.x = pose_std[0];
  pose.pose.position.y = pose_std[1];
  pose.pose.position.z = pose_std[2];
  pose.pose.orientation.x = pose_std[3];
  pose.pose.orientation.y = pose_std[4];
  pose.pose.orientation.z = pose_std[5];
  pose.pose.orientation.w = pose_std[6];

  pose.header.frame_id = "torso_lift_link"; // Assumed
  pose.header.stamp = ros::Time(0);
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "folding_client");
  manipulation::FoldingClient client;
  client.runExperiment();
  return 0;
}

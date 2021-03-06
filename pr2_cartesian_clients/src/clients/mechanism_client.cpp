#include <clients/mechanism_client.hpp>

using namespace pr2_cartesian_clients;
namespace manipulation{

MechanismClient::MechanismClient()
{
  nh_ = ros::NodeHandle("~");
  if(!loadParams())
  {
    ros::shutdown();
    return;
  }

  gravity_compensation_client_ = nh_.serviceClient<std_srvs::Empty>(gravity_compensation_service_name_);
  logging_service_client_ = nh_.serviceClient<pr2_cartesian_clients::LogMessages>(logging_service_);
  mechanism_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MechanismIdentificationAction>(mechanism_action_name_, true);
  move_action_client_ = new actionlib::SimpleActionClient<pr2_cartesian_controllers::MoveAction>(move_action_name_, true);
  action_server_ = new actionlib::SimpleActionServer<pr2_cartesian_clients::MechanismAction>(nh_, cartesian_client_action_name_, false);
  right_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
  left_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
  action_server_->registerGoalCallback(boost::bind(&MechanismClient::goalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&MechanismClient::preemptCB, this));
  feedback_thread_ = boost::thread(&MechanismClient::publishFeedback, this);
  current_iter_ = 0;
  alpha_granularity_ = 0;
}

MechanismClient::~MechanismClient()
{
  action_server_->shutdown();
  destroyActionClients();

  if (feedback_thread_.joinable())
  {
    feedback_thread_.interrupt();
    feedback_thread_.join();
  }
}

void MechanismClient::destroyActionClients()
{
  if (mechanism_action_client_)
  {
    mechanism_action_client_->cancelAllGoals();
    delete mechanism_action_client_;
    mechanism_action_client_ = nullptr;
  }

  if (move_action_client_)
  {
    move_action_client_->cancelAllGoals();
    delete move_action_client_;
    move_action_client_ = nullptr;
  }

  if (right_gripper_client_)
  {
    delete right_gripper_client_;
    right_gripper_client_ = nullptr;
  }

  if (left_gripper_client_)
  {
    delete left_gripper_client_;
    left_gripper_client_ = nullptr;
  }
}

bool MechanismClient::loadParams()
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
    ROS_ERROR("No base link frame name defined (experiment/base_link_name)");
    return false;
  }

  if(!nh_.getParam("experiment/use_estimates", use_estimates_))
  {
    ROS_ERROR("No use_estimates defined (experiment/use_estimates)");
    return false;
  }

  if(!nh_.getParam("experiment/use_nullspace", use_nullspace_))
  {
    ROS_ERROR("No use_nullspace defined (experiment/use_nullspace)");
    return false;
  }

  if(!nh_.getParam("experiment/nullspace_gain", km_))
  {
    ROS_ERROR("No nullspace_gain defined (experiment/nullspace_gain)");
    return false;
  }

  if(!nh_.getParam("experiment/init_t_error", init_t_error_))
  {
    ROS_ERROR("No initial translational dof error defined (experiment/init_t_error)");
    return false;
  }

  if(!nh_.getParam("experiment/init_k_error", init_k_error_))
  {
    ROS_ERROR("No initial rotational dof error defined (experiment/init_k_error)");
    return false;
  }
  
  if(!nh_.getParam("experiment/init_pc_error", init_pc_error_))
  {
    ROS_ERROR("No initial contact point estimation error defined (experiment/init_pc_error)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/mechanism_action_name", mechanism_action_name_))
  {
    ROS_ERROR("No mechanism identification controller action name defined (initialization/actionlib_server_names/mechanism_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/actionlib_server_names/move_action_name", move_action_name_))
  {
    ROS_ERROR("No move action name defined (initialization/actionlib_server_names/move_action_name)");
    return false;
  }

  if(!nh_.getParam("initialization/controller_names/mechanism_controller", mechanism_controller_name_))
  {
    ROS_ERROR("No mechanism action name defined (initialization/controller_names/mechanism_controller)");
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

  if(!nh_.getParam("initialization/controller_timeouts/mechanism_timeout", mechanism_action_time_limit_))
  {
    ROS_ERROR("No mechanism timeout defined (initialization/controller_timeouts/mechanism_timeout)");
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

  if(!nh_.getParam("experiment/vd_amp", vd_amp_))
  {
    ROS_ERROR("Need to set vd_amp (experiment/vd_amp)");
    return false;
  }

  if(!nh_.getParam("experiment/vd_freq", vd_freq_))
  {
    ROS_ERROR("Need to set vd_freq (experiment/vd_freq)");
    return false;
  }

  if(!nh_.getParam("experiment/wd_amp", wd_amp_))
  {
    ROS_ERROR("Need to set wd_amp (experiment/wd_amp)");
    return false;
  }

  if(!nh_.getParam("experiment/wd_freq", wd_freq_))
  {
    ROS_ERROR("Need to set wd_freq (experiment/wd_freq)");
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
    for (unsigned long i = 0; i < exclusion_list.size(); i++)
    {
      ROS_INFO("Adding exclusion: %s", exclusion_list[i].c_str());
      controller_runner_.addException(exclusion_list[i]);
    }
  }

  return true;
}

void MechanismClient::publishFeedback()
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

void MechanismClient::preemptCB()
{
  ROS_WARN("The mechanism client was preempted!");
  action_server_->setPreempted();
  current_action_.clear();
}

void MechanismClient::goalCB()
{
  boost::lock_guard<boost::mutex> guard(reference_mutex_);
  boost::shared_ptr<const pr2_cartesian_clients::MechanismGoal> goal = action_server_->acceptNewGoal();

  if (!action_server_->isPreemptRequested())
  {
    ROS_INFO("%s received a new goal!", cartesian_client_action_name_.c_str());

    if (goal->use_goal)
    {
      ROS_INFO("Using goal");
      mechanism_action_time_limit_ = goal->mechanism_timeout;
      num_of_experiments_ = goal->num_of_experiments;
      bag_prefix_ = goal->bag_prefix.data;
      vd_amp_ = goal->vd_amp;
      vd_freq_ = goal->vd_freq;
      wd_amp_ = goal->wd_amp;
      wd_freq_ = goal->wd_freq;
      goal_force_ = goal->goal_force;
      rod_arm_ = goal->rod_arm;
      surface_arm_ = goal->surface_arm;
      use_nullspace_ = goal->use_nullspace;
      use_estimates_ = goal->use_estimates;
      init_t_error_ = goal->init_t_error;
      init_k_error_ = goal->init_k_error;
      init_pc_error_ = goal->init_pc_error;
      km_ = goal->nullspace_gain;
      scale_alpha_ = goal->scale_alpha;
      alpha_granularity_ = goal->alpha_granularity;

      if (goal->randomize_initial_error)
      {
        noise_t_ = std::uniform_real_distribution<double>(goal->t_min, goal->t_max);
        noise_k_ = std::uniform_real_distribution<double>(goal->k_min, goal->k_max);
        noise_pc_ = std::uniform_real_distribution<double>(goal->pc_min, goal->pc_max);
      }
      else
      {
        noise_t_ = std::uniform_real_distribution<double>(0, 0);
        noise_k_ = std::uniform_real_distribution<double>(0, 0);
        noise_pc_ = std::uniform_real_distribution<double>(0, 0);
      }
    }
    else
    {
      noise_t_ = std::uniform_real_distribution<double>(0, 0);
      noise_k_ = std::uniform_real_distribution<double>(0, 0);
      noise_pc_ = std::uniform_real_distribution<double>(0, 0);
    }
  }
  else
  {
    ROS_WARN("Received a new goal, but a pending preempt was requested. Will ignore");
    action_server_->setPreempted();
  }
}

void MechanismClient::openGripper(GripperClient* gripper_client)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.10;
  open.command.max_effort = 50.0;

  ROS_INFO("Sending open goal");
  gripper_client->sendGoal(open);
  gripper_client->waitForResult();
}

void MechanismClient::closeGripper(GripperClient* gripper_client)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = 50.0;

  ROS_INFO("Sending squeeze goal");
  gripper_client->sendGoal(squeeze);
  gripper_client->waitForResult();
}

void MechanismClient::softGripper(GripperClient* gripper_client)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal soft;
  soft.command.position = 0.0;
  soft.command.max_effort = 0.0;

  ROS_INFO("Sending soft goal");
  gripper_client->sendGoal(soft);
}

void MechanismClient::runExperiment()
{
  ros::Time init, curr;
  bool got_eef_pose = false;
  int current_iter = 1, num_alpha_iter = 1;

  action_server_->start();
  ROS_INFO("Started the mechanism client action server: %s", cartesian_client_action_name_.c_str());
  ROS_INFO("Moving to initial configuration");

  while (ros::ok())
  {
    if (action_server_->isActive())
    {
      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_); // to wait if goal is being processed
      }

      pr2_cartesian_controllers::MoveGoal move_goal;
      pr2_cartesian_controllers::MechanismIdentificationGoal mechanism_goal;

      ROS_INFO("Starting experiment!");
      current_iter = 1;

      move_goal.desired_pose.resize(2);
      move_goal.is_single_arm = false;

      {
        boost::lock_guard<boost::mutex> guard(reference_mutex_);
        current_action_ = move_action_name_;
      }

      move_goal.desired_pose[rod_arm_] = initial_rod_pose_;
      move_goal.desired_pose[surface_arm_] = initial_surface_pose_;

      if (!controller_runner_.runController(move_controller_name_))
      {
        ROS_ERROR("Failed to run the controller %s", move_action_name_.c_str());
        action_server_->setAborted();
        continue;
      }
      
      sleep(1.5);
      controller_runner_.stopController("l_arm_controller");
      controller_runner_.stopController("r_arm_controller");

      bool move_timeout = false;
      if (!monitorActionGoal<pr2_cartesian_controllers::MoveAction,
                            pr2_cartesian_controllers::MoveGoal,
                            pr2_cartesian_clients::MechanismAction>
                              (move_action_client_, move_goal, action_server_, server_timeout_, move_action_time_limit_, move_timeout))
      {
        ROS_ERROR("Error in the move action. Aborting.");
        action_server_->setAborted();
        continue;
      }
      ROS_INFO("Move action succeeded!");

      // Zero the ft sensor readings
      std_srvs::Empty srv;
      for (int i = 0; i < 5; i++)
      {
        if(!gravity_compensation_client_.call(srv))
        {
          ROS_ERROR("Error calling the gravity compensation server!");
          action_server_->setAborted();
          return;
        }
        sleep(1.0);
      }

      openGripper(left_gripper_client_);
      openGripper(right_gripper_client_);
      softGripper(left_gripper_client_);
      softGripper(right_gripper_client_);
      ROS_INFO("Place mechanism");
      std::cin.get();
      closeGripper(left_gripper_client_);
      closeGripper(right_gripper_client_);
      double alpha = 0.5;
      
      if (scale_alpha_)
      {
        ROS_INFO("Scalling alpha!");
        num_alpha_iter = alpha_granularity_;
        alpha = 0;
      }
      int total_iter = 0;
      for (int i = 0; i <= alpha_granularity_ && action_server_->isActive(); i++)
      {
        while(action_server_->isActive() && current_iter <= num_of_experiments_)
        {
          feedback_.alpha = alpha;
          total_iter++;
          // Send rod arm to right initial pose
          {
            boost::lock_guard<boost::mutex> guard(reference_mutex_);
            current_action_ = move_action_name_;
          }
          
          move_goal.desired_pose[rod_arm_] = initial_rod_pose_;
          move_goal.desired_pose[surface_arm_] = initial_surface_pose_;
          
          if (!controller_runner_.runController(move_controller_name_))
          {
            ROS_ERROR("Failed to run the controller %s", move_action_name_.c_str());
            action_server_->setAborted();
            continue;
          }
          
          // sleep(1.5);
          // controller_runner_.stopController("l_arm_controller");
          // controller_runner_.stopController("r_arm_controller");
          
          bool move_timeout = false;
          if (!monitorActionGoal<pr2_cartesian_controllers::MoveAction,
            pr2_cartesian_controllers::MoveGoal,
            pr2_cartesian_clients::MechanismAction>
            (move_action_client_, move_goal, action_server_, server_timeout_, move_action_time_limit_, move_timeout))
            {
              ROS_ERROR("Error in the move action. Aborting.");
              action_server_->setAborted();
              continue;
            }
          ROS_INFO("Move action succeeded!");

          // if(!gravity_compensation_client_.call(srv))
          // {
          //   ROS_ERROR("Error calling the gravity compensation server!");
          //   // action_server_->setAborted();
          //   // return;
          // }
          
          {
            boost::lock_guard<boost::mutex> guard(reference_mutex_);
            current_action_ = mechanism_action_name_;
            current_iter_ = current_iter;
          }
          // controller_runner_.runController("l_arm_controller");
          // controller_runner_.runController("r_arm_controller");
          
          if (!controller_runner_.runController(mechanism_controller_name_))
          {
            ROS_ERROR("Failed to run the controller %s", mechanism_action_name_.c_str());
            action_server_->setAborted();
            continue;
          }
          
          // sleep(1.5);
          // controller_runner_.stopController("l_arm_controller");
          // controller_runner_.stopController("r_arm_controller");
          
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
          std::string bag_name;
          bag_name = std::string(bag_prefix_) + std::string("_") + std::to_string(total_iter);
          ROS_INFO("Running experiment %d/%d", current_iter, num_of_experiments_);
          {
            pr2_cartesian_clients::LogMessages srv;
            srv.request.log_type = srv.request.START_LOGGING;
            srv.request.name = bag_name;
            srv.request.max_record_time = mechanism_action_time_limit_ + 10;
            if (!logging_service_client_.call(srv))
            {
              ROS_WARN("Error calling the logging service, will not be able to log experiment");
            }
          }
            
          mechanism_goal.rod_arm = rod_arm_;
          mechanism_goal.alpha = alpha;
          mechanism_goal.surface_arm = surface_arm_;
          mechanism_goal.vd_amplitude = vd_amp_;
          mechanism_goal.wd_amplitude = wd_amp_;
          mechanism_goal.vd_frequency = vd_freq_;
          mechanism_goal.wd_frequency = wd_freq_;
          mechanism_goal.goal_force = goal_force_;
          mechanism_goal.use_nullspace = use_nullspace_;
          mechanism_goal.use_estimates = use_estimates_;
          mechanism_goal.nullspace_gain = km_;
          mechanism_goal.init_t_error = init_t_error_ + noise_t_(noise_generator_);
          mechanism_goal.init_k_error = init_k_error_ + noise_k_(noise_generator_);
          mechanism_goal.init_pc_error = init_pc_error_ + noise_pc_(noise_generator_);
          
          bool mechanism_timeout = false;
          if (!monitorActionGoal<pr2_cartesian_controllers::MechanismIdentificationAction,
            pr2_cartesian_controllers::MechanismIdentificationGoal,
            pr2_cartesian_clients::MechanismAction>
            (mechanism_action_client_, mechanism_goal, action_server_, server_timeout_, mechanism_action_time_limit_, mechanism_timeout))
            {
              if (!mechanism_timeout)
              {
                ROS_ERROR("Error in the mechanism action.");
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
          if (scale_alpha_)
          {
            alpha += (double) 1/alpha_granularity_;
          }
          ros::spinOnce();
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000/feedback_hz_));
        }
        alpha = 0;
        current_iter = 1;
      }
      ROS_INFO("Experiment done!");
      controller_runner_.unloadAll();
      openGripper(left_gripper_client_);
      softGripper(left_gripper_client_);
      openGripper(right_gripper_client_);
      softGripper(right_gripper_client_);
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

bool MechanismClient::getPose(const std::string &param, geometry_msgs::PoseStamped &pose)
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
  ros::init(argc, argv, "mechanism_client");
  manipulation::MechanismClient client;
  client.runExperiment();
  return 0;
}

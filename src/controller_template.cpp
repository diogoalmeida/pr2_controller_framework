#include <pr2_cartesian_controllers/controller_template.hpp>

namespace cartesian_controllers {
  template <class ActionClass, class ActionFeedback, class ActionResult>
  ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::ControllerTemplate()
  {
    nh_ = ros::NodeHandle("~");

    if(!loadParams() || !loadGenericParams())
    {
      ros::shutdown();
      exit(0);
    }

    // Initialize KDL variables
    joint_positions_.resize(7);
    kdl_parser::treeFromUrdfModel(model_, tree_); // convert URDF description of the robot into a KDL tree
    tree_.getChain(base_link_, end_effector_link_, chain_);
    fkpos_ = new KDL::ChainFkSolverPos_recursive(chain_);
    ikvel_ = new KDL::ChainIkSolverVel_wdls(chain_, eps_);

    // Subscribe to force and torque measurements
    ft_sub_ = nh_.subscribe(ft_topic_name_, 1, &ControllerTemplate::forceTorqueCB, this);

    // Initialize actionlib server
    action_server_ = new actionlib::SimpleActionServer<ActionClass>(nh_, action_name_, false);

    // Register callbacks
    action_server_->registerGoalCallback(boost::bind(&ControllerTemplate::goalCB, this));
    action_server_->registerPreemptCallback(boost::bind(&ControllerTemplate::preemptCB, this));

    action_server_->start();

    boost::thread(boost::bind(&ControllerTemplate::publishFeedback, this));
  }

  /*
    Update current force and torque values.
  */
  template <class ActionClass, class ActionFeedback, class ActionResult>
  void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    geometry_msgs::Vector3Stamped vector_in, vector_out;
    geometry_msgs::Wrench transformed_wrench;
    boost::lock_guard<boost::mutex> guard(reference_mutex_);

    vector_in.vector = msg->wrench.torque;
    vector_in.header = msg->header;
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.torque = vector_in.vector;

    vector_in.vector = msg->wrench.force;
    listener_.transformVector(base_link_, vector_in, vector_out);
    transformed_wrench.force = vector_in.vector;

    tf::wrenchMsgToEigen(transformed_wrench, measured_wrench_);
  }

  /*
    Dummy feedback publisher
  */
  template <class ActionClass, class ActionFeedback, class ActionResult>
  void ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::publishFeedback()
  {
    ROS_WARN("%s does not implement a feedback publisher", action_name_.c_str());
  }

  /*
    Search for generic controller parameters
  */
  template <class ActionClass, class ActionFeedback, class ActionResult>
  bool ControllerTemplate<ActionClass, ActionFeedback, ActionResult>::loadGenericParams()
  {
    if (!nh_.getParam("/common/end_effector_link_name", end_effector_link_))
    {
      ROS_ERROR("Missing end-effector link name (/common/end_effector_link_name)");
      return false;
    }

    if (!nh_.getParam("/common/base_link_name", base_link_))
    {
      ROS_ERROR("Missing base link name (/common/base_link_name)");
      return false;
    }

    if (!nh_.getParam("/common/wdls_epsilon", eps_))
    {
      ROS_ERROR("Missing wdls epsilon (/common/wdls_epsilon)");
      return false;
    }

    if (!nh_.getParam("/common/feedback_rate", feedback_hz_))
    {
      ROS_ERROR("Missing feedback_rate (/common/feedback_rate)");
      return false;
    }

    if(!model_.initParam("/robot_description")){
        ROS_ERROR("ERROR getting robot description (/robot_description)");
        return false;
    }

    return true;
  }
}

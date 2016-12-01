#ifndef __CARTESIAN_CLIENTS_EXTRAS__
#define __CARTESIAN_CLIENTS_EXTRAS__

#include <ros/ros.h>
#include <string>
#include <vector>
#include <algorithm>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

namespace pr2_cartesian_clients{
  bool stringInVector(std::string s, std::vector<std::string> v);

  /*
    This method monitors the goal state of the client and returns true if it succeeds.
    If a preempt is requested in the server, it cancels the client goal and returns false.
    It returns false if the client aborts.
  */
  template<typename ClientAction, typename Goal, typename ServerAction>
  bool monitorActionGoal(actionlib::SimpleActionClient<ClientAction> *client, Goal &goal, actionlib::SimpleActionServer<ServerAction> *server, double wait_timeout, double run_timeout)
  {
    ros::Time init, curr;
    if(!client->waitForServer(ros::Duration(wait_timeout)))
    {
      ROS_ERROR("Failed to connect to the action server");
      server->setAborted();
      return false;
    }

    client->sendGoal(goal);
    init = ros::Time::now();
    curr = ros::Time::now();
    while ((curr - init).toSec() < run_timeout)
    {
      if (!server->isActive()) // got preempted
      {
        client->cancelAllGoals();
        return false;
      }

      if (client->getState().isDone())
      {
        if (client->getState() == client->getState().SUCCEEDED)
        {
          return true;
        }

        return false;
      }
      ros::spinOnce();
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      curr = ros::Time::now();
    }

    if (!client->getState().isDone())
    {
      client->cancelAllGoals();
      return false;
    }
  }
}

#endif

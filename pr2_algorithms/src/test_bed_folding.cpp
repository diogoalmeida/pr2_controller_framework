#include <ros/ros.h>
#include <pr2_algorithms/folding_assembly_estimator.hpp>
#include <pr2_algorithms/TestBedFoldingFeedback.h>
#include <pr2_cartesian_controllers/FoldingControllerAction.h>
#include <cstdlib>
#include <random>

using namespace manipulation_algorithms;

void feedbackCallback(const pr2_cartesian_controllers::FoldingControllerActionFeedback &msg)
{
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d ret;

  ret << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;

  return ret;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");

  ros::NodeHandle n;
  FoldingAssemblyEstimator estimator_alg;
  Eigen::Vector3d pc, p2, r_real, r_real_p2, r_hat, pc_hat, force, torque;
  pr2_algorithms::TestBedFoldingFeedback feedback_msg;
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  ros::Rate r(1000);
  std::default_random_engine generator;
  bool use_real_data = false;
  double max_time, epsilon;
  std::normal_distribution<double> obs_noise(0.0, 0.0);

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedFoldingFeedback>("/test_bed/feedback", 1);
  // ros::Subscriber sub = n.subscribe("/realtime_loop/dexterous_manipulation/feedback", 1, feedbackCallback);

  if (argc > 1)
  {
    max_time = atof(argv[1]);
  }
  else
  {
    max_time = 10;
  }

  if (argc > 2)
  {
    use_real_data = true;
  }

  r_real << 0.07, 0.01, -0.1; // in world frame
  r_real_p2 << 0.07, -0.01, 0.1;
  p2 << 1, 0, -1; // in world frame
  pc = r_real + p2;
  r_hat << 0, 0, 0;
  pc_hat = Eigen::Vector3d::Zero();

  estimator_alg.getParams(n);
  estimator_alg.initialize(r_hat);

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();

  int i = 0;
  while (elapsed.toSec() < max_time)
  {
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;
    i++;

    if (elapsed.toSec() < max_time / 4)
    {
      force << 1 + 1*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator), 2 + 1.0*std::sin(3*M_PI*0.01*i) + 0.01*obs_noise(generator), 3 + 1.0*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator); // in p2 frame
      // force << 1 + 0.01*obs_noise(generator), 2 + 0.01*obs_noise(generator), 3 + 0.01*obs_noise(generator); // in p2 frame
    }
    else
    {
      if (elapsed.toSec() < max_time / 2)
      {
        force << -0.2 + 0.2*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator), -1 + 0.1*std::sin(3*M_PI*0.01*i) + 0.01*obs_noise(generator), 5 +2.0*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator); // in p2 frame
        // force << -0.2 + 0.01*obs_noise(generator), -1 + 0.01*obs_noise(generator), 5 + 0.01*obs_noise(generator); // in p2 frame
      }
      else
      {
        if (elapsed.toSec() < 3*max_time / 4)
        {
          force << 3 + 0.4*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator), 0.1 + 3.0*std::sin(3*M_PI*0.01*i) + 0.01*obs_noise(generator), 2 +0.01*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator); // in p2 frame
          // force << 3 + 0.01*obs_noise(generator), 0.1 + 0.01*obs_noise(generator), 2 + 0.01*obs_noise(generator); // in p2 frame
        }
        else
        {
          force << 0.7 + 2*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator), -0.01 + 1.1*std::sin(3*M_PI*0.01*i) + 0.01*obs_noise(generator), 1 +0.1*std::sin(2*M_PI*0.01*i) + 0.01*obs_noise(generator); // in p2 frame
          // force << 0.7 + 0.01*obs_noise(generator), -0.01 + 0.01*obs_noise(generator), 1 + 0.01*obs_noise(generator); // in p2 frame
        }
      }
    }

    torque = -skew(force)*r_real_p2;// + 0.01*obs_noise(generator);

    if (!use_real_data)
    {
    }
    else
    {
    }

    std::cout << "r_hat: " << std::endl << r_hat << std::endl << std::endl;
    // std::cout << -skew(force) << std::endl;
    std::cout << "innov: " << std::endl << torque + skew(force)*r_hat << std::endl << std::endl;

    r_hat = estimator_alg.estimate(Eigen::Vector3d::Zero(), force, torque, dt.toSec());

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    ros::spinOnce();
    r.sleep();
  }
}

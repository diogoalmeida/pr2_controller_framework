#include <ros/ros.h>
#include <pr2_algorithms/manipulation_control_algorithm.hpp>
#include <pr2_algorithms/manipulation_ekf.hpp>
#include <pr2_algorithms/TestBedFeedback.h>
#include <cstdlib>
#include <random>

using namespace manipulation_algorithms;
const double k_s = 0.2;
const double L = 0.12;

void getInitialState(Eigen::Vector3d &x_c, Eigen::Vector3d &x_e, double &spring)
{
  double init_theta = 0.9;
  double init_force = 0.4;
  double init_x = 0.1;

  x_c << init_x, init_theta, init_force;
  spring = -init_force*L/k_s;
  x_e << init_x + L*std::cos(init_theta), L*std::sin(init_theta), spring + init_theta;
}

Eigen::Matrix3d computeG(const double x_e, const double x_c, const double theta_c)
{
  Eigen::Matrix3d g;
  double d_x, eps;

  eps = std::numeric_limits<double>::epsilon();
  d_x = x_e - x_c;

  if (std::abs(1/cos(theta_c)) < eps || std::abs(d_x) < eps) // prevent 0/0, defined as 0
  {
    ROS_WARN("Limit");
    std::cout << cos(theta_c) << std::endl;
    std::cout << theta_c << std::endl;
    std::cout << d_x << std::endl;

    g << 1, 0, 0,
         0, 0, 0,
         0, 0, 0;
  }
  else
  {
    g << 1, std::tan(theta_c)              , 0                   ,
         0, 1/d_x                     , 0                   ,
         0, k_s*std::cos(theta_c)/(d_x*d_x), -k_s*std::cos(theta_c)/d_x;
  }

  return g;
}

void updateSpring(double &spring, const double d_theta_e, const double d_theta_c, const double dt)
{
  double spring_vel = d_theta_e - d_theta_c;

  spring = spring + spring_vel*dt;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");

  ros::NodeHandle n;
  ManipulationAlgorithm control_alg;
  ManipulationEKF estimator_alg;
  Eigen::Vector3d x_c, d_xc, x_e, u, x_d, variances;
  Eigen::VectorXd x_hat, y;
  pr2_algorithms::TestBedFeedback feedback_msg;
  Eigen::Matrix3d G;
  bool spring_est = false;
  double max_time, epsilon, spring, force, torque;
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  ros::Rate r(1000);
  std::default_random_engine generator;
  std::normal_distribution<double> obs_noise(0.0, 0.01);

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedFeedback>("/test_bed/feedback", 1);

  x_d = Eigen::Vector3d::Zero();
  x_d[0] = 0.02;
  x_d[1] = 0.1;
  x_d[2] = -1; // desired contact force
  force = 0;
  torque = 0;
  getInitialState(x_c, x_e, spring);
  estimator_alg.getParams(n);
  control_alg.getParams(n);

  if (argc > 1)
  {
    max_time = atof(argv[1]);

    if (argc > 2)
    {
      spring_est = true;
      x_hat = Eigen::VectorXd(4);
      y = Eigen::VectorXd(3);
      x_hat << x_c[0] + 0.4, x_c[1] - 0.4, x_c[2] - 0.1, k_s - 0.05;
    }
    else
    {
      x_hat = Eigen::VectorXd(3);
      y = Eigen::VectorXd(3);
      x_hat << x_c[0] + 0.1, x_c[1] + 0.2, x_c[2] + 0.5;
    }
  }
  else
  {
    max_time = 10;
  }

  estimator_alg.initialize(x_hat);
  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();

  while (elapsed.toSec() < max_time)
  {
    // u = control_alg.compute(x_d, x_c, x_e);
    // u = control_alg.compute(x_d, x_hat, x_e);

    G = computeG(x_e[0], x_c[0], x_c[1]);
    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;


    if (elapsed.toSec() < 0.25*max_time)
    {
      u[0] = 0;
      u[1] = 0;
      u[2] = 0.1*std::sin(0.25*2*M_PI*elapsed.toSec()) + 0.05*obs_noise(generator);
    }
    else
    {
      if (elapsed.toSec() < 0.5*max_time)
      {
        u[0] = 0.1*std::sin(2*M_PI*elapsed.toSec()) + 0.01*obs_noise(generator);
        u[1] = 0.005*std::sin(2*M_PI*elapsed.toSec()/4) + 0.01*obs_noise(generator);
        u[2] = 0.1*std::sin(0.25*2*M_PI*elapsed.toSec()) + 0.05*obs_noise(generator);
      }
      else
      {
        if (elapsed.toSec() < 0.75*max_time)
        {
          u[1] = 0.005*std::sin(2*M_PI*elapsed.toSec()/4) + 0.01*obs_noise(generator);
          u[0] = -u[1]*x_e[1]/(x_e[0] - x_c[0]) + 0.01*obs_noise(generator);
          u[2] = 0;
        }
        else
        {
          u[0] = 0;
          u[1] = 0;
          u[2] = 0.1*std::sin(0.25*2*M_PI*elapsed.toSec()) + 0.05*obs_noise(generator);
        }
      }
    }

    x_e = x_e + u*dt.toSec();
    d_xc = G*u;
    x_c = x_c + d_xc*dt.toSec();

    updateSpring(spring, u[2], d_xc[1], dt.toSec());
    torque = -k_s*spring;
    force = -torque/L;

    // ROS_INFO("spring: %f\nforce: %f\ntorque: %f", spring, force, torque);

    if(std::abs(k_s*std::cos(x_c[1])) > epsilon)
    {
      if (!spring_est)
      {
        y << torque/force,  x_e[2] + torque/k_s, force;
        // y << (x_e[0] - x_c[0])/std::cos(x_c[1]) + 0*obs_noise(generator), x_c[1] - x_c[2]*(x_e[0] - x_c[0])/(k_s*std::cos(x_c[1])) + 0*obs_noise(generator), x_c[2] + 0*obs_noise(generator);
      }
      else
      {
        // y << torque/force + obs_noise(generator), x_e[2], x_c[2] + 0.1*obs_noise(generator), torque + 0.05*obs_noise(generator);
        y << torque/force + obs_noise(generator), x_e[2], force + 0.1*obs_noise(generator);
      }
    }
    else
    {
      ROS_WARN("Division by zero");
      y << 0, x_e[2], force;
    }

    std::cout << "y:" << std::endl << y << std::endl;

    variances = estimator_alg.getVariance();
    std::cout << variances << std::endl << std::endl;
    x_hat = estimator_alg.estimate(u, y, x_e, dt.toSec());

    feedback_msg.x_c = x_c[0];
    feedback_msg.theta_c = x_c[1];
    feedback_msg.f_c = force;


    feedback_msg.x_e = x_e[0];
    feedback_msg.y_e = x_e[1];
    feedback_msg.theta_e = x_e[2];

    feedback_msg.x_c_hat = x_hat[0];
    feedback_msg.theta_c_hat = x_hat[1];
    feedback_msg.f_c_hat = x_hat[2];

    feedback_msg.var_x = x_hat[0] + 3*variances[0];
    feedback_msg.var_theta = x_hat[1] + 3*variances[1];
    feedback_msg.var_f = x_hat[2] + 3*variances[2];

    feedback_msg.k_s = k_s;
    feedback_msg.tau_e = torque;

    if (spring_est)
    {
      feedback_msg.k_s_hat = x_hat[3];
    }

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    r.sleep();
  }
}

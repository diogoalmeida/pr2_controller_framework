#include <ros/ros.h>
#include <pr2_algorithms/manipulation_control_algorithm.hpp>
#include <pr2_algorithms/manipulation_ekf.hpp>
#include <pr2_algorithms/TestBedFeedback.h>
#include <cstdlib>
#include <random>

using namespace manipulation_algorithms;
const double k_s = 0.18;
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

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");

  ros::NodeHandle n;
  ManipulationAlgorithm control_alg;
  ManipulationEKF estimator_alg;
  Eigen::Vector3d x_c, d_xc, x_e, u, x_d, y, variances;
  Eigen::VectorXd x_hat(4);
  pr2_algorithms::TestBedFeedback feedback_msg;
  Eigen::Matrix3d G;
  double k_s, max_time, epsilon, spring, force, torque;
  ros::Time init_time, prev_time;
  ros::Duration dt;
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
  x_hat << x_c[0], x_c[1], x_c[2], k_s;
  estimator_alg.getParams(n);
  control_alg.getParams(n);
  estimator_alg.initialize(x_hat);

  if (argc > 1)
  {
    max_time = atof(argv[1]);
  }
  else
  {
    max_time = 10;
  }

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  epsilon = std::numeric_limits<double>::epsilon();

  while ((ros::Time::now() - init_time).toSec() < max_time)
  {
    // u = control_alg.compute(x_d, x_c, x_e);
    u = control_alg.compute(x_d, x_hat, x_e);
    // u = Eigen::Vector3d::Zero();
    G = computeG(x_e[0], x_c[0], x_c[1]);
    dt = ros::Time::now() - prev_time;
    x_e = x_e + u*dt.toSec();
    d_xc = G*u;
    x_c = x_c + d_xc*dt.toSec();
    spring = x_e[2] - x_c[1];
    torque = -k_s*spring;
    force = torque/L;

    // ROS_INFO("spring: %f\nforce: %f\ntorque: %f", spring, force, torque);

    if(std::abs(k_s*std::cos(x_c[1])) > epsilon)
    {
      // y << torque/force + 0*obs_noise(generator),  x_e[1] + torque/k_s + 0*obs_noise(generator), force + 0*obs_noise(generator);
      y << L + 0*obs_noise(generator), x_c[1] + 0*obs_noise(generator), x_c[2] + 0*obs_noise(generator);
      // y << (x_e[0] - x_c[0])/std::cos(x_c[1]) + 0*obs_noise(generator), x_c[1] - x_c[2]*(x_e[0] - x_c[0])/(k_s*std::cos(x_c[1])) + 0*obs_noise(generator), x_c[2] + 0*obs_noise(generator);
    }
    else
    {
      y << 0, x_c[1] + 0*obs_noise(generator), force + 0*obs_noise(generator);
    }

    std::cout << "y:" << std::endl << y << std::endl;

    variances = estimator_alg.getVariance();
    std::cout << variances << std::endl << std::endl;
    x_hat = estimator_alg.estimate(u, y, x_e, dt.toSec());

    feedback_msg.x_c = x_c[0];
    feedback_msg.theta_c = x_c[1];
    feedback_msg.f_c = x_c[2];


    feedback_msg.x_e = x_e[0];
    feedback_msg.y_e = x_e[1];
    feedback_msg.theta_e = x_e[2];

    feedback_msg.x_c_hat = x_hat[0];
    feedback_msg.theta_c_hat = x_hat[1];
    feedback_msg.f_c_hat = x_hat[2];

    feedback_msg.var_x = x_hat[0] + 3*variances[0];
    feedback_msg.var_theta = x_hat[1] + 3*variances[1];
    feedback_msg.var_f = x_hat[2] + 3*variances[2];

    pub.publish(feedback_msg);
    r.sleep();
  }
}

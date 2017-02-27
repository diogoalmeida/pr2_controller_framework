#include <ros/ros.h>
#include <pr2_algorithms/manipulation_control_algorithm.hpp>
#include <pr2_algorithms/manipulation_ekf.hpp>
#include <pr2_algorithms/TestBedFeedback.h>
#include <pr2_cartesian_controllers/ManipulationControllerAction.h>
#include <cstdlib>
#include <random>

using namespace manipulation_algorithms;
const double k_s = 0.17;
const double L = 0.12;
Eigen::Vector3d real_u, real_x_e, real_x_c;
double real_f_y = 0;
double real_f_x = 0;
double real_torque = 0;

void getInitialState(Eigen::VectorXd &x_c, Eigen::Vector3d &x_e, double &spring)
{
  double init_theta = 0.6;
  double init_force = -0.6;
  double init_x = 0.1;

  x_c << init_x, init_theta, init_force;
  spring = -init_force*L*std::cos(init_theta)/k_s;
  x_e << init_x + L*std::cos(init_theta), L*std::sin(init_theta), spring + init_theta;
}

Eigen::MatrixXd computeG(const double x_e, const double x_c, const double theta_c, const double theta_e, const double f_c_x, const double f_c_y)
{
  Eigen::MatrixXd g(3, 3);
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
         0, (k_s + d_x*std::tan(theta_c)*f_c_y)/(d_x*d_x), -k_s/d_x;
  }

  return g;
}

void updateSpring(double &spring, const double d_theta_e, const double d_theta_c, const double dt)
{
  double spring_vel = d_theta_e - d_theta_c;

  spring = spring + spring_vel*dt;
}

void feedbackCallback(const pr2_cartesian_controllers::ManipulationControllerActionFeedback &msg)
{
  real_u << msg.feedback.x_e_dot, msg.feedback.y_e_dot, msg.feedback.theta_e_dot;
  real_f_y = msg.feedback.f_c_y;
  real_f_x = msg.feedback.f_c_x;
  real_torque = msg.feedback.torque_c;
  real_x_e << msg.feedback.x_e, msg.feedback.y_e, msg.feedback.theta_e;
  real_x_c << msg.feedback.x_c_2, msg.feedback.theta_c_2, real_f_y;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_bed");

  ros::NodeHandle n;
  ManipulationAlgorithm control_alg;
  ManipulationEKF estimator_alg;
  Eigen::Vector3d x_e, u, x_d, variances;
  Eigen::VectorXd x_hat, y, x_c, d_xc(3);
  pr2_algorithms::TestBedFeedback feedback_msg;
  Eigen::MatrixXd G(3, 3);
  double max_time, epsilon, spring, force, torque;
  ros::Time init_time, prev_time;
  ros::Duration dt, elapsed;
  ros::Rate r(1000);
  std::default_random_engine generator;
  bool use_real_data = false;
  std::normal_distribution<double> obs_noise(0.0, 0.01);

  ros::Publisher pub = n.advertise<pr2_algorithms::TestBedFeedback>("/test_bed/feedback", 1);
  ros::Subscriber sub = n.subscribe("/realtime_loop/dexterous_manipulation/feedback", 1, feedbackCallback);

  x_d = Eigen::Vector3d::Zero();
  x_d[0] = 0.02;
  x_d[1] = 0.1;
  x_d[2] = -1; // desired contact force
  force = 0;
  torque = 0;
  x_c = Eigen::VectorXd(3);
  getInitialState(x_c, x_e, spring);
  estimator_alg.getParams(n);
  control_alg.getParams(n);

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

  real_u = Eigen::Vector3d::Zero();
  real_x_e = Eigen::Vector3d::Zero();
  x_hat = Eigen::VectorXd(4);
  y = Eigen::VectorXd(3);


  while (use_real_data && ros::Time::now() == ros::Time(0))
  {
    r.sleep();
  }
  ros::spinOnce();

  if (!use_real_data)
  {
    x_hat << x_c[0] - 0.1, x_c[1] - 0.3, x_c[2] - 0.3, k_s - 0.1;
  }
  else
  {
    x_hat << real_x_e[0] - 0.1, real_x_e[2], real_f_y, k_s;
  }

  estimator_alg.initialize(x_hat);

  init_time = ros::Time::now();
  prev_time = ros::Time::now();
  elapsed = ros::Time::now() - init_time;
  epsilon = std::numeric_limits<double>::epsilon();
  Eigen::VectorXd x_c_aug(3);

  while (elapsed.toSec() < max_time)
  {
    u = control_alg.compute(x_d, x_hat, x_e);

    dt = ros::Time::now() - prev_time;
    elapsed = ros::Time::now() - init_time;

    if (elapsed.toSec() < 0.25*max_time)
    {
      u[0] = 0.1*std::sin(2*M_PI*elapsed.toSec()) + 0.01*obs_noise(generator);
      u[1] = 0;
      u[1] = 0.005*std::sin(2*M_PI*elapsed.toSec()/4) + 0.01*obs_noise(generator);
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
          u[0] = 0.1*std::sin(2*M_PI*elapsed.toSec()) + 0.01*obs_noise(generator);
          u[2] = 0.1*std::sin(0.25*2*M_PI*elapsed.toSec()) + 0.05*obs_noise(generator);
        }
        else
        {
          u[0] = 0.1*std::sin(2*M_PI*elapsed.toSec()) + 0.01*obs_noise(generator);
          u[1] = 0;
          u[1] = 0.005*std::sin(2*M_PI*elapsed.toSec()/4) + 0.01*obs_noise(generator);
          u[2] = 0.1*std::sin(0.25*2*M_PI*elapsed.toSec()) + 0.05*obs_noise(generator);
        }
      }
    }
    // u = control_alg.compute(x_d, x_hat, x_e);

    G = computeG(x_e[0], x_c[0], x_c[1], x_e[2], 0, force);

    if (!use_real_data)
    {
      x_e = x_e + u*dt.toSec();
      d_xc = G*u;
      updateSpring(spring, u[2], d_xc[1], dt.toSec());
    }
    else
    {
      x_e = x_e + real_u*dt.toSec();
      d_xc = G*real_u;
      updateSpring(spring, real_u[2], d_xc[1], dt.toSec());
    }

    x_c = x_c + d_xc*dt.toSec();

    torque = -k_s*spring;// + 0.01*obs_noise(generator);
    force = torque/(L*std::cos(x_c[1]));// + 0.05*obs_noise(generator);

    if (!use_real_data)
    {
      y << torque/force + 0*0.05*obs_noise(generator), x_e[2], force + 0*10*obs_noise(generator);
      x_hat = estimator_alg.estimate(u, y, x_e, dt.toSec());
    }
    else
    {
      // y << real_torque, real_x_e[2] + real_torque/k_s, 0, -std::sqrt(real_f_y*real_f_y + real_f_x*real_f_x);
      y << real_torque/real_f_y, real_x_e[2], real_f_y;
      // y << torque + 5*obs_noise(generator), x_e[2] + torque/k_s, 0, force + 10*obs_noise(generator);
      // x_hat = estimator_alg.estimate(real_u, y, x_e, dt.toSec());
      x_hat = estimator_alg.estimate(real_u, y, real_x_e, dt.toSec());
    }

    feedback_msg.f_x_sim = 0;
    feedback_msg.f_y_sim = x_c[2] + 0*10*obs_noise(generator);
    feedback_msg.tau_e_sim = torque;

    feedback_msg.f_x_real = real_f_x;
    feedback_msg.f_y_real = real_f_y;
    feedback_msg.tau_e_real = real_torque;
    feedback_msg.x_c_sim = x_c[0];
    feedback_msg.x_c_real = real_x_c[0];
    feedback_msg.theta_c_sim = x_c[1];
    feedback_msg.theta_c_real = real_x_c[1];
    feedback_msg.f_spring = force;

    feedback_msg.x_e = x_e[0];
    feedback_msg.y_e = x_e[1];
    feedback_msg.theta_e = x_e[2];

    feedback_msg.x_e_dot = u[0];
    feedback_msg.y_e_dot = u[1];
    feedback_msg.theta_e_dot = u[2];

    feedback_msg.x_c_hat = x_hat[0];
    feedback_msg.theta_c_hat = x_hat[1];
    feedback_msg.f_x_hat = 0;
    feedback_msg.f_y_hat = x_hat[2];

    feedback_msg.k_s = k_s;
    feedback_msg.k_s_hat = x_hat[3];

    prev_time = ros::Time::now();
    pub.publish(feedback_msg);
    ros::spinOnce();
    r.sleep();
  }
}

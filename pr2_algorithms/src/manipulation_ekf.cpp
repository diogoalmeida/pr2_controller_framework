#include <pr2_algorithms/manipulation_ekf.hpp>

namespace manipulation_algorithms{
  ManipulationEKF::ManipulationEKF()
  {
    x_hat_ << Eigen::Vector3d::Zero();
    P_ << Eigen::Matrix3d::Identity();
    Q_ << Eigen::Matrix3d::Identity();
    R_ << Eigen::Matrix3d::Identity();
    k_s_ = 1; // avoid division by 0
    theta_o_ = 0;
  }

  ManipulationEKF::~ManipulationEKF()
  {
    // Bye!
  }

  void ManipulationEKF::initialize(const Eigen::Vector3d &init_x)
  {
    x_hat_ = init_x;

    P_ << Eigen::Matrix3d::Identity();
  }

  bool ManipulationEKF::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/manipulation_controller/spring_constant", k_s_))
    {
      ROS_ERROR("Missing spring constant (/manipulation_controller/spring_constant)");
      return false;
    }

    if (!n.getParam("/manipulation_controller/estimate_spring_constant", estimate_k_s_)) // if true, k_s_ will be taken as a state variable
    {
      ROS_ERROR("Missing estimate spring constant (/manipulation_controller/estimate_spring_constant)");
      return false;
    }

    if (estimate_k_s_)
    {
      P_ = Eigen::MatrixXd(4,4);
      R_ = Eigen::MatrixXd(4,4);
    }
    else
    {
      P_ = Eigen::MatrixXd(3,3);
      P_ = Eigen::MatrixXd(3,3);
    }

    if (!n.getParam("/manipulation_controller/initial_angle_offset", theta_o_))
    {
      ROS_ERROR("Missing initial angle offset (/manipulation_controller/initial_angle_offset)");
      return false;
    }

    if(!parseMatrixData(Q_, "/manipulation_controller/estimator/Q", n))
    {
      return false;
    }

    if(!parseMatrixData(R_, "/manipulation_controller/estimator/R", n))
    {
      return false;
    }

    return true;
  }

  Eigen::Vector3d ManipulationEKF::getVariance()
  {
    Eigen::Vector3d variances;

    variances << P_(0, 0), P_(1,1), P_(2,2);

    return variances;
  }

  Eigen::Vector3d ManipulationEKF::estimate(const Eigen::Vector3d &u, const Eigen::Vector3d &y, const Eigen::Vector3d &x_e, const double dt)
  {
    Eigen::Matrix3d A, C, K, G, I, P, O;
    Eigen::Vector3d h, innovation;
    double dx, dx_square, dx_cube, cos_theta, cos_theta_square, sin_theta, tan_theta, epsilon;

    I = Eigen::Matrix3d::Identity();
    dx = x_e[0] - x_hat_[0];
    dx_square = dx*dx;
    dx_cube = dx_square*dx;
    cos_theta = cos(x_hat_[1]);
    sin_theta = sin(x_hat_[1]);
    tan_theta = tan(x_hat_[1]);
    cos_theta_square = cos_theta*cos_theta;
    epsilon = std::numeric_limits<double>::epsilon();

    if (std::abs(dx_cube) < epsilon || std::abs(cos_theta) < epsilon || std::abs(cos_theta_square) < epsilon)
    {
      ROS_WARN("Division by 0. dx_cube: %f\ncos_theta: %f\ncos_theta_square: %f\nepsilon: %f", dx_cube, cos_theta, cos_theta_square, epsilon);
      return x_hat_;
    }

    A << 0                         , u[1]/cos_theta_square, 0,
         u[1]/dx_square, 0          , 0,
         k_s_*cos_theta*(2*u[1] - u[2]*dx)/dx_cube, k_s_*sin_theta*(u[1] - u[2]*dx)/dx_square, 0;

    C << -1/cos_theta, tan_theta*dx/cos_theta, 0,
         0, 0, 1,
         0, 1, 0;

    G << 1, tan_theta, 0,
         0, 1/dx, 0,
         0, k_s_*cos_theta/dx_square, -k_s_*cos_theta/dx;

    h << dx/cos_theta, x_hat_[2], x_hat_[1];

    innovation = y - h;

    // 1 - predict according to the end-effector motion (u)
    x_hat_ = x_hat_ + G*u*dt;
    A = I + A*dt;
    P_.triangularView<Eigen::Upper>() = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;
    // P_ = A*P_*A.transpose() + R_;

    // 2 - update based on the innovation
    K = P_.selfadjointView<Eigen::Upper>()*C.transpose()*(C*P_.selfadjointView<Eigen::Upper>()*C.transpose() + Q_).inverse();
    // K = P_*C.transpose()*(C*P_*C.transpose() + Q_).inverse();
    x_hat_ = x_hat_ + K*innovation;
    P_.triangularView<Eigen::Upper>() = (I - K*C)*P_.selfadjointView<Eigen::Upper>();
    // P_ = (I - K*C)*P_;

    return x_hat_;
  }

}

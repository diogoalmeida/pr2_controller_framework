#include <pr2_algorithms/manipulation_ekf.hpp>

namespace manipulation_algorithms{
  ManipulationEKF::ManipulationEKF()
  {
    k_s_ = 1; // avoid division by 0
    theta_o_ = 0;
  }

  ManipulationEKF::~ManipulationEKF()
  {
    // Bye!
  }

  void ManipulationEKF::initialize(const Eigen::VectorXd &init_x)
  {
    x_hat_ = Eigen::VectorXd(init_x.rows());
    for (int i = 0; i < init_x.rows(); i++)
    {
      x_hat_[i] = init_x[i];
    }

    for (int i = 0; i < init_x.rows()*init_x.rows(); i++)
    {
      P_(i) = 0;
    }

    P_(0, 0) = 0.001;
    P_(1, 1) = 0.001;
    P_(2, 2) = 0.001;
  }

  bool ManipulationEKF::getParams(const ros::NodeHandle &n)
  {
    if (!n.getParam("/manipulation_controller/spring_constant", k_s_))
    {
      ROS_ERROR("Missing spring constant (/manipulation_controller/spring_constant)");
      return false;
    }

    P_ = Eigen::MatrixXd(3,3);

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

    if (R_.rows() != 3)
    {
      ROS_ERROR("Incorrect size for R, must be 5x5, is %dx%d", (int)R_.rows(), (int)R_.cols());
      return false;
    }

    return true;
  }

  void ManipulationEKF::initializeMatrices(int dim, Eigen::MatrixXd &A, Eigen::MatrixXd &C, Eigen::MatrixXd &G, Eigen::MatrixXd &I, Eigen::MatrixXd &K, Eigen::MatrixXd &P)
  {
    P = Eigen::MatrixXd(dim, dim);
    A = Eigen::MatrixXd(dim, dim);
    I = Eigen::MatrixXd(dim, dim);
    for (int i = 0; i < dim*dim; i++)
    {
      I(i) = 0;
      P(i) = 0;
    }
    for (int i = 0; i < dim; i++)
    {
      I(i, i) = 1;
      P(i, i) = 1;
    }

    C = Eigen::MatrixXd(3, 3);
    G = Eigen::MatrixXd(3, 3);
    K = Eigen::MatrixXd(3, 3);
  }

  void ManipulationEKF::computeA(Eigen::MatrixXd &A, const double y_e_dot, const double theta_e_dot, const double x_e, const double x_c, const double theta_c, const double f_c_y, const double f_c_x, const double k_s)
  {
    double tan_theta = std::tan(theta_c);
    double xi = tan_theta*f_c_y + f_c_x;
    double cos_theta = std::cos(theta_c);
    double dx = x_e - x_c;

    A << 0, y_e_dot/(cos_theta*cos_theta), 0,
         y_e_dot/(dx*dx), 0, 0,
         (tan_theta*f_c_y - k_s*theta_e_dot)/(dx*dx) - k_s*y_e_dot/(dx*dx*dx), f_c_y*y_e_dot/(dx*cos_theta*cos_theta), tan_theta*y_e_dot/dx;
  }

  void ManipulationEKF::computeC(Eigen::MatrixXd &C, const double x_e, const double x_c, const double theta_c, const double f_c_y, const double f_c_x, const double k_s)
  {
    double tan_theta = std::tan(theta_c);
    double cos_theta = std::cos(theta_c);
    double gamma = f_c_y - tan_theta*f_c_x;
    double dx = x_e - x_c;

    C << -1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  }

  void ManipulationEKF::computeG(Eigen::MatrixXd &G, const double x_e, const double x_c, const double theta_c, const double theta_e, const double f_c_x, const double f_c_y, const double k_s)
  {
    double dx, cos_theta, sin_theta, tan_theta;
    dx = x_e - x_c;
    cos_theta = std::cos(theta_c);
    sin_theta = std::sin(theta_c);
    tan_theta = sin_theta/cos_theta;

    G << 1, std::tan(theta_c)              , 0                   ,
         0, 1/dx                     , 0                   ,
         0, (k_s + dx*tan_theta*f_c_y)/(dx*dx), -k_s/dx;
  }

  Eigen::VectorXd ManipulationEKF::estimate(const Eigen::Vector3d &u, const Eigen::VectorXd &y, const Eigen::Vector3d &x_e, const double dt)
  {
    Eigen::MatrixXd A, C, P, G, I, K;
    Eigen::VectorXd h, innovation;
    double dx, dx_square, dx_cube, cos_theta, cos_theta_square, sin_theta, tan_theta, epsilon, xi, gamma_1, gamma_2;

    initializeMatrices(3, A, C, G, I, K, P);
    h = Eigen::VectorXd(3);

    dx = x_e[0] - x_hat_[0];
    dx_square = dx*dx;
    dx_cube = dx_square*dx;
    cos_theta = cos(x_hat_[1]);
    sin_theta = sin(x_hat_[1]);
    tan_theta = tan(x_hat_[1]);
    cos_theta_square = cos_theta*cos_theta;
    epsilon = std::numeric_limits<double>::epsilon();

    if (std::abs(dx_cube) < epsilon || std::abs(cos_theta) < epsilon || std::abs(cos_theta_square) < epsilon || std::abs(k_s_) < epsilon)
    {
      ROS_WARN("Division by 0. dx_cube: %f\ncos_theta: %f\ncos_theta_square: %f\nk_s: %f\nepsilon: %f", dx_cube, cos_theta, cos_theta_square, k_s_, epsilon);
      return x_hat_;
    }

    computeA(A, u[1], u[2], x_e[0], x_hat_[0], x_hat_[1], x_hat_[2], 0, k_s_);
    computeC(C, x_e[0], x_hat_[0], x_hat_[1], x_hat_[2], 0, k_s_);
    computeG(G, x_e[0], x_hat_[0], x_hat_[1], x_e[2], 0, x_hat_[2], k_s_);

    h << dx, x_hat_[1], x_hat_[2];

    innovation = y - h;

    // 1 - predict according to the end-effector motion (u)
    x_hat_ = x_hat_ + G*u*dt;
    A = I + A*dt;

    // P_.triangularView<Eigen::Upper>() = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;
    P_ = A*P_*A.transpose() + R_;

    // 2 - update based on the innovation
    // K = P_.selfadjointView<Eigen::Upper>()*C.transpose()*(C*P_.selfadjointView<Eigen::Upper>()*C.transpose() + Q_).inverse();
    K = P_*C.transpose()*(C*P_*C.transpose() + Q_).inverse();
    x_hat_ = x_hat_ + K*innovation;
    // P_.triangularView<Eigen::Upper>() = (I - K*C)*P_.selfadjointView<Eigen::Upper>();
    P_ = (I - K*C)*P_;

    // k_s_ = k_s_;

    return x_hat_;
  }

}

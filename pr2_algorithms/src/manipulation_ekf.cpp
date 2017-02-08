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
    for (int i = 0; i < 3; i++)
    {
      x_hat_[i] = init_x[i];
    }

    for (int i = 0; i < P_.rows(); i++)
    {
      P_(i, i) = 1;
    }

    if (init_x.size() > 3)
    {
      k_s_ = init_x[3];
    }
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
    }
    else
    {
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

    if (estimate_k_s_ && R_.rows() != 4)
    {
      ROS_ERROR("Incorrect size for R, must be 4x4, is %dx%d", (int)R_.rows(), (int)R_.cols());
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

  Eigen::VectorXd ManipulationEKF::estimate(const Eigen::Vector3d &u, const Eigen::Vector3d &y, const Eigen::Vector3d &x_e, const double dt)
  {
    Eigen::MatrixXd A, C, P, G, I, K;
    Eigen::Vector3d h, innovation;
    Eigen::VectorXd estimate;
    double dx, dx_square, dx_cube, cos_theta, cos_theta_square, sin_theta, tan_theta, epsilon, xi, gamma_1, gamma_2;

    if (estimate_k_s_)
    {
      C = Eigen::MatrixXd(3,4);
      P = Eigen::MatrixXd(4,4);
      G = Eigen::MatrixXd(4,3);
      A = Eigen::MatrixXd(4,4);
      K = Eigen::MatrixXd(4,3);
      estimate = Eigen::VectorXd(4);
      estimate << x_hat_[0], x_hat_[1], x_hat_[2], k_s_;
      I = Eigen::MatrixXd(4,4);
      for (int i = 0; i < 16; i++)
      {
        I(i) = 0;
      }

      for (int i = 0; i < 4; i++)
      {
        I(i, i) = 1;
      }
    }
    else
    {
      C = Eigen::MatrixXd(3,3);
      P = Eigen::MatrixXd(3,3);
      G = Eigen::MatrixXd(3,3);
      I = Eigen::MatrixXd(3,3);
      A = Eigen::MatrixXd(3,3);
      K = Eigen::MatrixXd(3,3);

      for (int i = 0; i < 9; i++)
      {
        I(i) = 0;
      }

      for (int i = 0; i < 3; i++)
      {
        I(i, i) = 1;
      }
      estimate = Eigen::VectorXd(3);
      estimate << x_hat_[0], x_hat_[1], x_hat_[2];
    }

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

      if (estimate_k_s_)
      {
        Eigen::VectorXd estimate(4);
        estimate << x_hat_[0], x_hat_[1], x_hat_[2], k_s_;
        return estimate;
      }

      return x_hat_;
    }

    gamma_1 = (2*u[1] - u[2]*dx)/dx_cube;
    gamma_2 = (u[1] - u[2]*dx)/dx_square;
    xi = 1/(cos_theta*k_s_);

    if (!estimate_k_s_)
    {
      A << 0                         , u[1]/cos_theta_square, 0,
           u[1]/dx_square, 0          , 0,
           k_s_*cos_theta*gamma_1, -k_s_*sin_theta*gamma_2, 0;

      C << -1/cos_theta, tan_theta*dx/cos_theta, 0,
            0, 1, 0,
            0, 0, 1;

      G << 1, tan_theta, 0,
           0, 1/dx, 0,
           0, k_s_*cos_theta/dx_square, -k_s_*cos_theta/dx;

      h << dx/cos_theta, x_hat_[1], x_hat_[2];
    }
    else
    {
      A << 0                         , u[1]/cos_theta_square, 0, 0,
           u[1]/dx_square, 0          , 0, 0,
           k_s_*cos_theta*gamma_1, -k_s_*sin_theta*gamma_2, 0, cos_theta*gamma_2,
           0             , 0          , 0, 0;

      C << -1/cos_theta, tan_theta*dx/cos_theta, 0, 0,
            -x_hat_[2]*xi, 1 + tan_theta*dx*x_hat_[2]*xi, dx*xi, -x_hat_[2]*dx*xi/k_s_,
            0, 0, 1, 0;

      G << 1, tan_theta, 0,
           0, 1/dx, 0,
           0, k_s_*cos_theta/dx_square, -k_s_*cos_theta/dx,
           0, 0, 0;

      h << dx/cos_theta, x_hat_[1] - x_hat_[2]*dx/(k_s_*cos_theta), x_hat_[2];
    }

    innovation = y - h;

    // std::cout << "innovation:" << std::endl << innovation << std::endl;

    // 1 - predict according to the end-effector motion (u)
    estimate = estimate + G*u*dt;
    A = I + A*dt;

    P_.triangularView<Eigen::Upper>() = A*P_.selfadjointView<Eigen::Upper>()*A.transpose() + R_;

    // 2 - update based on the innovation
    K = P_.selfadjointView<Eigen::Upper>()*C.transpose()*(C*P_.selfadjointView<Eigen::Upper>()*C.transpose() + Q_).inverse();
    estimate = estimate + K*innovation;
    P_.triangularView<Eigen::Upper>() = (I - K*C)*P_.selfadjointView<Eigen::Upper>();

    // std::cout << "K:" << std::endl << K << std::endl;
    // std::cout << "estimate:" << std::endl << estimate << std::endl;

    x_hat_ << estimate[0], estimate[1], estimate[2];

    if (estimate_k_s_)
    {
      k_s_ = estimate[3];
    }

    return estimate;
  }

}

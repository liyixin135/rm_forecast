//
// Created by ljt666666 on 22-10-8.
//

#ifndef RM_FORECAST_KALMAN_FILTER_H
#define RM_FORECAST_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <rm_forecast/KfConfig.h>

namespace rm_forecast
{
    using EigenFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
    struct KalmanFilterMatrices
    {
        EigenFunc F;  // state transition matrix
        Eigen::MatrixXd H;  // measurement matrix
        Eigen::MatrixXd Q;  // process noise covariance matrix
        Eigen::MatrixXd R;  // measurement noise covariance matrix
        Eigen::MatrixXd P;  // error estimate covariance matrix
    };

    class KalmanFilter
    {
    public:
        explicit KalmanFilter(const KalmanFilterMatrices & matrices);
        KalmanFilter() = default;

        // Initialize the filter with a guess for initial states.
        void init(const Eigen::VectorXd & x0);

        void setState(const Eigen::VectorXd& x0);

        // Computes a predicted state
        Eigen::MatrixXd predict();

        // Update the estimated state based on measurement
        Eigen::MatrixXd update(const Eigen::VectorXd & z);

        void initReconfigure();

    private:
        void reconfigCB(rm_forecast::KfConfig& config, uint32_t level)
        {
          if (!dynamic_reconfig_initialized_)
          {
            config.q_element = 0;
            config.q_value = Q.diagonal()(0);
            config.r_element = 0;
            config.r_value = R.diagonal()(0);
            dynamic_reconfig_initialized_ = true;
          }
          if (q_element_last != config.q_element)
          {
            q_element_last = config.q_element;
            config.q_value = Q.diagonal()(config.q_element);
          }
          if (r_element_last != config.r_element)
          {
            r_element_last = config.r_element;
            config.r_value = R.diagonal()(config.r_element);
          }
          Q.diagonal()(config.q_element) = config.q_value;
          R.diagonal()(config.r_element) = config.r_value;
        };

        // Invariant matrices
        EigenFunc F;
        Eigen::MatrixXd H, Q, R;

        // Priori error estimate covariance matrix
        Eigen::MatrixXd P_pre;
        // Posteriori error estimate covariance matrix
        Eigen::MatrixXd P_post;

        // Kalman gain
        Eigen::MatrixXd K;

        // System dimensions
        int n;

        // N-size identity
        Eigen::MatrixXd I;

        // Predicted state
        Eigen::VectorXd x_pre;
        // Updated state
        Eigen::VectorXd x_post;

        // Reconfigure
        dynamic_reconfigure::Server<rm_forecast::KfConfig>* reconf_server_;
        int dynamic_reconfig_initialized_{};
        int q_element_last;
        int r_element_last;
    };

}  // namespace rm_forecast

#endif //RM_FORECAST_KALMAN_FILTER_H

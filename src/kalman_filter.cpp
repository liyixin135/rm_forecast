//
// Created by ljt666666 on 22-10-8.
//
#include "kalman_filter.h"

namespace rm_forecast
{
    KalmanFilter::KalmanFilter(const KalmanFilterMatrices & matrices)
            : F(matrices.F),
              H(matrices.H),
              Q(matrices.Q),
              R(matrices.R),
              P_post(matrices.P),
              n(matrices.F.rows()),
              I(Eigen::MatrixXd::Identity(n, n)),
              x_pre(n), /***初始化一个还没赋值的矩阵***/
              x_post(n)
    {
    }

    void KalmanFilter::init(const Eigen::VectorXd & x0) { x_post = x0; } /***赋值上一个时刻的估计值***/

    Eigen::MatrixXd KalmanFilter::predict(const Eigen::MatrixXd & F)
    {
        this->F = F;

        x_pre = F * x_post;
        P_pre = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict 处理在下一次预测之前没有测量的情况
        x_post = x_pre;
        P_post = P_pre;

        return x_pre; /***返回本时刻的预测值***/
    }

    Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & z)
    {
        K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
        x_post = x_pre + K * (z - H * x_pre);
        P_post = (I - K * H) * P_pre;

        return x_post;
    }

    void KalmanFilter::initReconfigure()
    {
      // Init dynamic reconfigure
      reconf_server_ = new dynamic_reconfigure::Server<rm_forecast::KfConfig>(ros::NodeHandle("~/kf_config"));
      dynamic_reconfigure::Server<rm_forecast::KfConfig>::CallbackType cb = boost::bind(&KalmanFilter::reconfigCB, this, _1, _2);
      reconf_server_->setCallback(cb);
    }

    ExtendedKalmanFilter::ExtendedKalmanFilter(const rm_forecast::ExtendedKalmanFilterMatrices &matrices)
            : F(matrices.F),
              J_H(matrices.J_H),
              Q(matrices.Q),
              R(matrices.R),
              P_post(matrices.P),
              n(matrices.F.rows()),
              I(Eigen::MatrixXd::Identity(n, n)),
              x_pre(n), /***初始化一个还没赋值的矩阵***/
              x_post(n)
    {
    }

    void ExtendedKalmanFilter::init(const Eigen::VectorXd &x0) { x_post = x0; } /***赋值上一个时刻的估计值***/

    Eigen::MatrixXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd &F)
    {
        this->F = F;

        x_pre = F * x_post;
        P_pre = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict 处理在下一次预测之前没有测量的情况
        x_post = x_pre;
        P_post = P_pre;

        return x_pre; /***返回本时刻的预测值***/
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, const Eigen::MatrixXd & J_H)
    {
        this->J_H = J_H;

        K = P_pre * J_H.transpose() * (J_H * P_pre * J_H.transpose() + R).inverse();
        x_post = x_pre + K * (z - J_H * x_pre);
        P_post = (I - K * J_H) * P_pre;

        return x_post;
    }

}  // namespace rm_forecast
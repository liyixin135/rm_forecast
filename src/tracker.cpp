//
// Created by ljt666666 on 22-10-9.
//
#include "../include/forecast_node.h"
#include "../include/tracker.h"

namespace rm_forecast
{
    Tracker::Tracker(const KalmanFilterMatrices & kf_matrices)
            : tracker_state(LOST),
              tracking_id(0),
              kf_matrices_(kf_matrices),
              tracking_velocity_(Eigen::Vector3d::Zero())
    {
      kf_ = *new KalmanFilter(kf_matrices_);
    }

    void Tracker::init(const float &angle, const float &speed,
                       const float &q_speed) {
      // KF init
      Eigen::VectorXd init_state(3);
      //        const auto position = chosen_armor.pose.position;
      //        init_state << position.x, position.y, position.z, 0, 0, 0;
      init_state << angle, speed, q_speed;
      kf_.init(init_state);

      tracker_state =
          DETECTING; /***设置装甲板状态为目标识别中，需要更多帧识别到才开始跟踪***/
    }

    void Tracker::update(const float &angle,
                         const double &max_match_distance,
                         const int &tracking_threshold,
                         const int &lost_threshold) {
      // KF predict
      target_state = kf_.predict(); /***得出本时刻预测值***/

      bool matched = false;

      Eigen::VectorXd position_vec(1);
      position_vec << angle;
      /***只有这个时候才做卡尔曼滤波的更新部分***/
      target_state = kf_.update(position_vec);
      /***如果离预测目标最近的装甲板的距离大于阈值，则寻找另一个id相同的装甲板重新作为追踪目标，并把上一个追踪目标的速度给它***/
    }

}  // namespace rm_auto_aim
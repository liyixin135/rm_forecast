//
// Created by ljt666666 on 22-10-9.
//

#ifndef RM_FORECAST_TRACKER_H
#define RM_FORECAST_TRACKER_H

#include "kalman_filter.h"
#include "forecast_node.h"
#include "std_msgs/Float32.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <XmlRpcValue.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <rm_forecast/ForecastConfig.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <mutex>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <ros/ros.h>
#include <thread>
#include <vector>

namespace rm_forecast {

class Tracker {
public:
  Tracker(const KalmanFilterMatrices &kf_matrices);

  //        using Armors = auto_aim_interfaces::msg::Armors; /***？？***/
  //        using Armor = auto_aim_interfaces::msg::Armor;

  void init(const float &angle, const float &speed, const float &q_speed);

  void update(const float &angle, const float &speed, const float &q_speed,
              const double &dt, const double &max_match_distance,
              const int &tracking_threshold, const int &lost_threshold);

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  char tracking_id;
  Eigen::VectorXd target_state;

  double min_distance_ = 800;

  int detect_count_;
  int lost_count_;

private:
  KalmanFilterMatrices kf_matrices_;
  std::unique_ptr<KalmanFilter> kf_;

  Eigen::Vector3d tracking_velocity_;
};

class EKFTracker {
public:
    EKFTracker(const ExtendedKalmanFilterMatrices &ekf_matrices);

    //        using Armors = auto_aim_interfaces::msg::Armors; /***？？***/
    //        using Armor = auto_aim_interfaces::msg::Armor;

    void init(const float &a, const float &omega, const float &theta, const float &b);
    void init(const float &a, const float &faiz, const float &theta, const float &b, const float &omega);

    void update(const float &angle, const double &a, const double &omega, const double &theta, const double &b,
                const double &dt, const double &last_second,
                const double &max_match_distance, const int &tracking_threshold, const int &lost_threshold);
    void update(const float &speed, const double &a, const double &faiz,
                const double &theta, const double &b, const double &omega,
                const double &dt);

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
        } tracker_state;

    char tracking_id;
    Eigen::VectorXd target_state;

    double min_distance_ = 800;

    int detect_count_;
    int lost_count_;

private:
    ExtendedKalmanFilterMatrices ekf_matrices_;
    std::unique_ptr<ExtendedKalmanFilter> ekf_;

    Eigen::Vector3d tracking_velocity_;
};

} // namespace rm_forecast

#endif // RM_FORECAST_TRACKER_H

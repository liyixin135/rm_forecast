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

  void init(const float &angle, const float &speed, const float &q_speed, bool detect);

  void update(const double &angle, const double &max_match_distance,
              const int &tracking_threshold, const int &lost_threshold, bool detect);

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  Eigen::VectorXd target_state;

  int detect_count_;
  int lost_count_;

  KalmanFilter kf_;

private:
  KalmanFilterMatrices kf_matrices_;
  ros::Publisher debug_pub_;

  Eigen::Vector3d tracking_velocity_;
};
} // namespace rm_forecast

#endif // RM_FORECAST_TRACKER_H

//
// Created by ljt666666 on 22-10-9.
//

#pragma once

#include "kalman_filter.h"
#include "tracker.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <XmlRpcValue.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <mutex>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <rm_forecast/ForecastConfig.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <vector>

using namespace std;
namespace rm_forecast {

static double max_match_distance_{};
static int tracking_threshold_{};
static int lost_threshold_{};

class Forecast_Node : public nodelet::Nodelet {
public:

  Forecast_Node() = default;

  ~Forecast_Node() override {
    if (this->my_thread_.joinable())
      my_thread_.join();
  }

  void initialize(ros::NodeHandle &nh);

  void onInit() override;

private:
  rm_msgs::TargetDetectionArray target_array_;
  ros::Subscriber targets_sub_;
  ros::Publisher track_pub_;
  ros::NodeHandle nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  dynamic_reconfigure::Server<rm_forecast::ForecastConfig>*forecast_cfg_srv_{};
  dynamic_reconfigure::Server<rm_forecast::ForecastConfig>::CallbackType forecast_cfg_cb_;

  // transform
 //  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::Buffer *tf_buffer_;

  // Last time received msg
  ros::Time last_time_;

  // Initial KF matrices
  KalmanFilterMatrices kf_matrices_;
  double dt_;

  // Armor tracker
  std::unique_ptr<Tracker> tracker_;
  bool tracking_{};

  void forecastconfigCB(rm_forecast::ForecastConfig &config, uint32_t level);

  void speedCallback(const rm_msgs::TargetDetectionArray::Ptr &msg);

  std::thread my_thread_;
  image_transport::Publisher image_pub_;
};

} // namespace rm_forecast

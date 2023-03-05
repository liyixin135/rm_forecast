//
// Created by ljt666666 on 22-10-9.
//

#pragma once

#include "kalman_filter.h"
#include "rm_common/ori_tool.h"
#include "spin_observer.h"
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
static double max_jump_angle_{};
static double max_jump_period_{};
static double allow_following_range_{};

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
  dynamic_reconfigure::Server<rm_forecast::ForecastConfig> *forecast_cfg_srv_{};
  dynamic_reconfigure::Server<rm_forecast::ForecastConfig>::CallbackType
      forecast_cfg_cb_;

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

  // Spin observer
  std::unique_ptr<SpinObserver> spin_observer_;
  bool allow_spin_observer_ = true;

  void forecastconfigCB(rm_forecast::ForecastConfig &config, uint32_t level);

  void speedCallback(const rm_msgs::TargetDetectionArray::Ptr &msg);

  void outpostCallback(const rm_msgs::TargetDetectionArray::Ptr &msg);
  geometry_msgs::Pose
  computeCircleCenter(const rm_msgs::TargetDetection point_1,
                      const rm_msgs::TargetDetection point_2,
                      const rm_msgs::TargetDetection point_3,
                      const rm_msgs::TargetDetection point_4);
  bool forecast_readied_ = false;
  bool reset_ = false;
  bool init_flag_ = false;
  bool fitting_succeeded_ = false;
  int target_quantity_ = 0;
  int min_target_quantity_ = 100;
  double line_speed_ = 0.8;
  double z_c_ = 0.5;
  double outpost_radius_ = 0.35;
  double rotate_speed_ = 0.5;
  bool circle_suggest_fire_ = false;
  double time_offset_ = 0.53;
  double fly_time_ = 0.5;
  double y_thred_ = 0.05;
  double x_thred_ = 0.05;
  ros::Time last_min_time_;
  double max_x_ = 0, min_x_ = 0, min_distance_ = 0;
  rm_msgs::TargetDetectionArray max_x_target_;
  rm_msgs::TargetDetectionArray min_x_target_;
  rm_msgs::TargetDetection target_[4]{};
  rm_msgs::TargetDetectionArray min_distance_target_;
  ros::Time min_x_time_, last_get_target_time_;
  double theta_b_;
  int count_{};
  std::thread my_thread_;
  image_transport::Publisher image_pub_;
  double fly_time;
};

} // namespace rm_forecast

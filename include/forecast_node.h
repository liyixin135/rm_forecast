//
// Created by ljt666666 on 22-10-9.
//
#ifndef RM_FORECAST_FORECAST_H
#define RM_FORECAST_FORECAST_H

#pragma once

#include "kalman_filter.h"
#include "rm_common/filters/lp_filter.h"
#include "std_msgs/Float32.h"
#include "tracker.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <XmlRpcValue.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <mutex>
#include <nodelet/nodelet.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <pluginlib/class_loader.h>
#include <rm_forecast/ForecastConfig.h>
#include <rm_msgs/StatusChange.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <vector>
#include <random>

using namespace std;

namespace rm_forecast {
struct InfoTarget {
  ros::Time stamp;
  float angle;
};

struct Target {
  std::vector<float> points;
  cv::Point3f armor_center_points;
  cv::Point3f r_points;
  int label;
  Eigen::Matrix3d rmat;
  Eigen::Vector3d tvec;
};

struct finalTarget {
  ros::Time stamp;
  double speed;
  double x;
  double y;
  double z;
};

static double max_match_distance_{};
static int tracking_threshold_{};
static int lost_threshold_{};

class Forecast_Node : public nodelet::Nodelet {
public:
  Forecast_Node() : filter_(cutoff_frequency_) {}

  ~Forecast_Node() override {
    if (this->my_thread_.joinable())
      my_thread_.join();
  }

  void initialize(ros::NodeHandle &nh);

  void onInit() override;

  static Eigen::MatrixXd jacobianFunc(const Eigen::VectorXd &x, const double &dt, const double &last_second);
  static Eigen::MatrixXd jacobianFunc(const Eigen::VectorXd &x);
  void publishMarkers(const rm_msgs::TrackData& track_data);

private:
  rm_msgs::TargetDetectionArray target_array_;
  ros::Subscriber points_targets_sub_;
  ros::Publisher debug_pub_;
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

  bool dynamic_reconfig_initialized_ = false;
  void forecastconfigCB(rm_forecast::ForecastConfig &config, uint32_t level);

  std::thread my_thread_;
  image_transport::Publisher image_pub_;

  /// LowPassFilter
  LowPassFilter filter_;
  double cutoff_frequency_ = -1;

  /// draw reproject
  void drawCallback(const sensor_msgs::ImageConstPtr &img);

  template <typename T>
  bool initMatrix(Eigen::MatrixXd &matrix, std::vector<T> &vector);

  ros::Subscriber draw_sub_;
  image_transport::Publisher draw_pub_;

  bool is_reproject_;

  cv::Mat m_intrinsic_;

  bool updateFan(Target &object, const InfoTarget &prev_target);

  void speedSolution(InfoTarget &prev_target);

  float getAngle();

  bool speed_init_flag_ = false;
  bool skip_flag_ = false;
  Target prev_fan_{};
  Target last_fan_{};
  double speed_{};
  double last_kal_speed_{};
  double skip_frame_{};
  double skip_frame_threshold_{};
  double angle_{};
  double x_angle_{};
  double y_angle_{};
  double actual_frame_angle_{};
  double frame_angle_{};
  double theta_offset_{};

  bool changeStatusCB(rm_msgs::StatusChange::Request &change,
                      rm_msgs::StatusChange::Response &res);
  ros::ServiceServer status_change_srv_;
  bool is_small_buff_ = true;

  /// windmill kalman
  void pointsCallback(const rm_msgs::TargetDetectionArray::Ptr &msg);

  Target pnp(const std::vector<cv::Point2f> &points_pic);

  std::vector<double> calcAimingAngleOffset(Target &object, double params[4],
                                            double t0, double t1, int mode);

  cv::Point2f reproject(Eigen::Vector3d &xyz);

  cv::Matx33f cam_intrinsic_mat_k_;
  //    std::vector<double> dist_coefficients_;
  cv::Matx<float, 1, 5> dist_coefficients_;

  bool is_clockwise_;
  double fan_length_; // 大符臂长(R字中心至装甲板中心)
  double image_fan_length_;
  double target_length_;
  double target_width_;
  double angular_velocity_;
  double delay_time_;
  double speed_threshold_;
  double high_acceleration_coefficient_;
  double low_acceleration_coefficient_;
  double high_acceleration_offset_;
  double low_acceleration_offset_;

  double init_second_;
  double last_second_;
  double amplitude_;
  double angular_frequency_;
  double theta_;
  double offset_;
//  double faiz_;

  bool is_static_;

  int plus_num_ = 0;
  int minus_num_ = 0;
  float last_speed_ = 0;
  float last_a_ = 0;
  geometry_msgs::Point last_detection_temp_{};
  geometry_msgs::Vector3 last_velocity_temp_{};
  cv::Point2f target2d_{};
  cv::Point2f r2d_{};
  cv::Point2f last_target2d_{};
  cv::Point2f last_r2d_{};

  std::deque<finalTarget> history_info_;
  bool is_filled_up_ = false;
  int deque_len_ = 200;
  double max_delta_t_ = 0.3;

  // draw history target
  std::deque<std::pair<cv::Point2f,ros::Time>> target2d_his_;
  ros::Time stamp_;

  // Calculate the indefine-velocity
  vector<finalTarget> vel_buf_;

  // Visualization marker publisher
  visualization_msgs::Marker position_marker_;
  visualization_msgs::Marker linear_v_marker_;
  visualization_msgs::Marker angular_v_marker_;
  visualization_msgs::Marker armors_marker_;
  ros::Publisher marker_pub_;

};

} // namespace rm_forecast

#endif // RM_FORECAST_FORECAST_H
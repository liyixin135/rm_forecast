//
// Created by ljt666666 on 22-10-9.
//

#include "../include/forecast_node.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(rm_forecast::Forecast_Node, nodelet::Nodelet)

namespace rm_forecast {
void Forecast_Node::onInit() {
  ros::NodeHandle &nh = getMTPrivateNodeHandle();
  static ros::CallbackQueue my_queue;
  nh.setCallbackQueue(&my_queue);
  initialize(nh);
  my_thread_ = std::thread([]() {
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&my_queue);
  });
}

void Forecast_Node::initialize(ros::NodeHandle &nh) {
  nh_ = ros::NodeHandle(nh, "rm_forecast");
  it_ = make_shared<image_transport::ImageTransport>(nh_);

  ROS_INFO("starting ProcessorNode!");

  // Kalman Filter initial matrix
  // A - state transition matrix
  // clang-format off
        Eigen::Matrix<double, 6, 6> f;
        f <<  1,  0,  0, dt_, 0,  0,
                0,  1,  0,  0, dt_, 0,
                0,  0,  1,  0,  0, dt_,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;
  // clang-format on

  // H - measurement matrix
  Eigen::Matrix<double, 3, 6> h;
  h.setIdentity(); /***把矩阵左上角3x3赋值为对角为1其余为0***/

  // Q - process noise covariance matrix
  Eigen::DiagonalMatrix<double, 6> q;
  q.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;

  // R - measurement noise covariance matrix
  Eigen::DiagonalMatrix<double, 3> r;
  r.diagonal() << 0.05, 0.05, 0.05;

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 6> p;
  p.setIdentity();

  kf_matrices_ =
      KalmanFilterMatrices{f, h, q, r, p}; /***初始化卡尔曼滤波初始参数***/

  if(!nh.getParam("max_match_distance", max_match_distance_))
      ROS_WARN("No max match distance specified");
    ROS_INFO("66%lf", max_match_distance_);
  if(!nh.getParam("tracking_threshold", tracking_threshold_))
        ROS_WARN("No tracking threshold specified");
  if(!nh.getParam("lost_threshold", lost_threshold_))
        ROS_WARN("No lost threshold specified");

  tracker_ = std::make_unique<Tracker>(kf_matrices_);

  forecast_cfg_srv_ = new dynamic_reconfigure::Server<rm_forecast::ForecastConfig>(ros::NodeHandle(nh_, "rm_forecast"));
  forecast_cfg_cb_ = boost::bind(&Forecast_Node::forecastconfigCB, this, _1, _2);
  forecast_cfg_srv_->setCallback(forecast_cfg_cb_);

  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(10));
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  targets_sub_ =
      nh.subscribe("/detection", 1, &Forecast_Node::speedCallback, this);
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);
}

void Forecast_Node::forecastconfigCB(rm_forecast::ForecastConfig &config,
                                     uint32_t level) {
  //        target_type_ = config.target_color;
  max_match_distance_ = config.max_match_distance;
  tracking_threshold_ = config.tracking_threshold;
  lost_threshold_ = config.lost_threshold;

}

void Forecast_Node::speedCallback(
    const rm_msgs::TargetDetectionArray::Ptr &msg) {
  if (msg->detections.empty()) {
    ROS_INFO("no target!");
    return;
  }

   //Tranform armor position from image frame to world coordinate
  this->target_array_.detections.clear();
  target_array_.header = msg->header;
  for (const auto &detection : msg->detections) {
    rm_msgs::TargetDetection detection_temp;
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    pose_in.header.frame_id = msg->header.frame_id;
    pose_in.header.stamp = msg->header.stamp;
    pose_in.pose = detection.pose;

    try
    {
      geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
          "odom", pose_in.header.frame_id, msg->header.stamp, ros::Duration(1));

      tf2::doTransform(pose_in.pose, pose_out.pose, transform);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
//    ROS_INFO_STREAM(pose_out.pose.position.x
//                    << ",y:" << pose_out.pose.position.y
//                    << ",z:" << pose_out.pose.position.z);

    detection_temp.id = detection.id;
    detection_temp.confidence = detection.confidence;
    detection_temp.pose = pose_out.pose;
    target_array_.detections.emplace_back(detection_temp);
  }

  rm_msgs::TrackData track_data;

  /***如果是丢失状态则初始化tracker***/
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(target_array_);
    //            target_msg.tracking = false;
    tracking_ = false;
  }

  /***是其他状态则更新tracker***/
  else {
    // Set dt
    dt_ = (msg->header.stamp - last_time_).toSec();
    // Update state
    tracker_->update(target_array_, dt_, max_match_distance_, tracking_threshold_, lost_threshold_);
    /***判断是否处于跟踪状态 ***/
    if (tracker_->tracker_state == Tracker::DETECTING) {
      //                target_msg.tracking = false;
      tracking_ = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING ||
               tracker_->tracker_state == Tracker::TEMP_LOST) {
      //                target_msg.tracking = true;
      tracking_ = true;
      //                target_msg.id = tracker_->tracking_id;//??
    }
  }
  last_time_ = msg->header.stamp;

  if (tracking_) {
    //            target_msg.position.x = tracker_->target_state(0);
    //            target_msg.position.y = tracker_->target_state(1);
    //            target_msg.position.z = tracker_->target_state(2);
    //            target_msg.velocity.x = tracker_->target_state(3);
    //            target_msg.velocity.y = tracker_->target_state(4);
    //            target_msg.velocity.z = tracker_->target_state(5);
    track_data.header.frame_id = "odom";
    track_data.header.stamp = ros::Time::now(); //??
    track_data.id = tracker_->tracking_id;
    track_data.target_pos.x = tracker_->target_state(0);
    track_data.target_pos.y = tracker_->target_state(1);
    track_data.target_pos.z = tracker_->target_state(2);
    track_data.target_vel.x = tracker_->target_state(3);
    track_data.target_vel.y = tracker_->target_state(4);
    track_data.target_vel.z = tracker_->target_state(5);
  }
  track_pub_.publish(track_data);
}

} // namespace rm_forecast

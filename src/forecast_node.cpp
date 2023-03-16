//
// Created by ljt666666 on 22-10-9.
//

#include "../include/forecast_node.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

using namespace cv;
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

    if(!nh.getParam("max_jump_angle", max_jump_angle_))
        ROS_WARN("No max_jump_angle specified");
    if(!nh.getParam("max_jump_period", max_jump_period_))
        ROS_WARN("No max_jump_period_ specified");
    if(!nh.getParam("allow_following_range", allow_following_range_))
        ROS_WARN("No allow_following_range specified");

    if(!nh.getParam("y_thred", y_thred_))
        ROS_WARN("No y_thred specified");
    if(!nh.getParam("fly_time", fly_time_))
        ROS_WARN("No fly_time specified");
    if(!nh.getParam("allow_following_range", allow_following_range_))
        ROS_WARN("No allow_following_range specified");

  tracker_ = std::make_unique<Tracker>(kf_matrices_);

  spin_observer_ = std::make_unique<SpinObserver>();

  forecast_cfg_srv_ = new dynamic_reconfigure::Server<rm_forecast::ForecastConfig>(ros::NodeHandle(nh_, "rm_forecast"));
  forecast_cfg_cb_ = boost::bind(&Forecast_Node::forecastconfigCB, this, _1, _2);
  forecast_cfg_srv_->setCallback(forecast_cfg_cb_);

  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(10));
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  targets_sub_ =
      nh.subscribe("/detection", 1, &Forecast_Node::speedCallback, this);
//    targets_sub_ =
//            nh.subscribe("/detection", 1, &Forecast_Node::outpostCallback, this);
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);

    std::vector<float> intrinsic;
    std::vector<float> distortion;
    if(!nh.getParam("/forecast/camera_matrix/data", intrinsic))
        ROS_WARN("No cam_intrinsic_mat_k specified");
    if(!nh.getParam("/forecast/distortion_coefficients/data", distortion))
        ROS_WARN("No distortion specified");
    if(!nh.getParam("is_reproject", is_reproject_))
        ROS_WARN("No is_reproject specified");
    if(!nh.getParam("re_fly_time", re_fly_time_))
        ROS_WARN("No re_fly_time specified");

    Eigen::MatrixXd mat_intrinsic(3, 3);
    initMatrix(mat_intrinsic,intrinsic);
    eigen2cv(mat_intrinsic,m_intrinsic_);

    draw_sub_ = nh.subscribe("/galaxy_camera/image_raw", 1, &Forecast_Node::drawCallback, this);
    draw_pub_ = it_->advertise("reproject_image", 1);
}

void Forecast_Node::forecastconfigCB(rm_forecast::ForecastConfig &config,
                                     uint32_t level) {
//          target_type_ = config.target_color;
    ///track
  max_match_distance_ = config.max_match_distance;
  tracking_threshold_ = config.tracking_threshold;
  lost_threshold_ = config.lost_threshold;

  ///spin_observer
    max_jump_angle_ = config.max_jump_angle;
    max_jump_period_ = config.max_jump_period;
    allow_following_range_ = config.allow_following_range;

  ///outpost
    forecast_readied_ = config.forecast_readied;
    reset_ = config.reset;
    min_target_quantity_ = config.min_target_quantity;
    line_speed_ = config.line_speed;
    z_c_ = config.z_c;
    outpost_radius_ = config.outpost_radius;
    rotate_speed_ = config.rotate_speed;
    y_thred_ = config.y_thred;
    fly_time_ = config.fly_time;

    ///reproject
    is_reproject_ = config.is_reproject;
    re_fly_time_ = config.re_fly_time;

}

void Forecast_Node::outpostCallback(const rm_msgs::TargetDetectionArray::Ptr &msg) {
    rm_msgs::TrackData track_data;
    track_data.header.frame_id = "odom";
    track_data.header.stamp = target_array_.header.stamp;
    track_data.id = 0;

    if (msg->detections.empty()) {
        track_pub_.publish(track_data);
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

    if(forecast_readied_){
        if(reset_){
            reset_ = false;
            init_flag_ = false;
            fitting_succeeded_ = false;
            target_quantity_ = 0;
            min_y_ = 0;
            max_y_ = 0;
            min_distance_ = 0;
        }

        if(!init_flag_){
            init_flag_ = true;
            fitting_succeeded_ = false;
            max_y_target_ = target_array_;
            min_y_target_ = target_array_;
            min_distance_target_ = target_array_;
            return;
        }
        ROS_INFO("fitting%d", fitting_succeeded_);
        if(!fitting_succeeded_){
            if((abs(target_array_.detections[0].pose.position.y - min_y_) > 0.55) || (abs(target_array_.detections[0].pose.position.y - max_y_) > 0.55)){
                return;
            }

            if(target_array_.detections[0].pose.position.y > max_y_){
                max_y_ = target_array_.detections[0].pose.position.y;
                max_y_target_ = target_array_;
            }

            if(target_array_.detections[0].pose.position.y < min_y_){
                min_y_ = target_array_.detections[0].pose.position.y;
                min_y_target_ = target_array_;
            }

            if(target_array_.detections[0].pose.position.x < min_distance_){
                min_distance_ = target_array_.detections[0].pose.position.x;
                min_distance_target_ = target_array_;
            }
        }

        outpost_radius_ = 0.35;
        double line_speed = 6.28 * rotate_speed_ * outpost_radius_;
        ROS_INFO("line_speed:%f", line_speed_);
        double theta_b, delta_theta;
        if(target_quantity_ > min_target_quantity_){
            fitting_succeeded_ = true;
            ROS_INFO("max_y_ - min_y_:%f", (max_y_ - min_y_));
            ROS_INFO("outpost_radius_:%f", outpost_radius_);
            double theta_a = 2 * asin(0.5 * (max_y_ - min_y_) / outpost_radius_);
            theta_b = 0.5 * (3.14 - theta_a);
            double offset_y = -(outpost_radius_ * sin(theta_b_));
            min_y_time_ = min_y_target_.header.stamp;
            double theta_c = (target_array_.header.stamp - min_y_time_).toSec() * rotate_speed_;
//            x_c = -(outpost_radius_ * cos(theta_b_ + theta_c));
//            y_c = offset_y - (-(outpost_radius_ * sin(theta_b_ + theta_c)));
            ROS_INFO("outpost_radius_:%f", outpost_radius_);
            double offset = 0.5 * (max_y_ + min_y_);
            double amend_c = -target_array_.detections[0].pose.position.y + offset;
            delta_theta = acos(amend_c / outpost_radius_) - theta_b;
            ROS_INFO("fitting successed. theta_a:%f, theta_b:%f, delta_t:%f, offset:%f", theta_a, theta_b, delta_theta, offset);
        }
        else{
            ROS_INFO("fitting params. %d", target_quantity_);
            ++target_quantity_;
        }

        if(!fitting_succeeded_)
            return;

//        ROS_INFO("y_thred:%f", target_array_.detections[0].pose.position.y - min_distance_target_.detections[0].pose.position.y);
//        if(abs(target_array_.detections[0].pose.position.y - min_distance_target_.detections[0].pose.position.y) < y_thred_){
//            last_min_time_ = ros::Time::now();
//        }
//        double duration = (ros::Time::now() - last_min_time_).toSec();
//        if(duration > fly_time_)
//            circle_suggest_fire_ = true;

        geometry_msgs::PoseStamped pose_in;
        geometry_msgs::PoseStamped pose_out;
        pose_in.header.frame_id = "base_link";
        pose_in.header.stamp = msg->header.stamp;
        pose_in.pose = target_array_.detections[0].pose;
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
        target_array_.detections[0].pose = pose_out.pose;

//        ROS_INFO("theta_b + delta_theta %f", theta_b + delta_theta);
//        double min_distance_x, min_distance_y, min_distance_z;
//        if(abs(theta_b + delta_theta - 1.57) < y_thred_){
//            min_distance_x = target_array_.detections[0].pose.position.x;
//            min_distance_y = target_array_.detections[0].pose.position.y;
//            min_distance_z = target_array_.detections[0].pose.position.z;
//            last_min_time_ = ros::Time::now();
//        }
//        double duration = (ros::Time::now() - last_min_time_).toSec();
//        ROS_INFO("duration:%f", duration);
//
//        if(duration > fly_time_ && duration < fly_time_ + 0.03)
//            circle_suggest_fire_ = true;
//
//        if(circle_suggest_fire_){
//            track_data.id = 7;
//            circle_suggest_fire_ = false;
//        }
//        else
//            track_data.id = 0;

        track_data.id = 5;
        track_data.header.frame_id = "odom";
        track_data.target_pos.x = target_array_.detections[0].pose.position.x;
        track_data.target_pos.y = target_array_.detections[0].pose.position.y;
        track_data.target_pos.z = z_c_;
        track_data.target_vel.x = -(line_speed_ * sin(theta_b + delta_theta));
        track_data.target_vel.y = line_speed_ * cos(theta_b + delta_theta);
        track_data.target_vel.z = 0;
//        track_data.target_pos.x = min_distance_x;
//        track_data.target_pos.y = min_distance_y;
//        track_data.target_pos.z = min_distance_z;
//        track_data.target_vel.x = 0;
//        track_data.target_vel.y = 0;
//        track_data.target_vel.z = 0;
//        ROS_INFO("circle_suggest_fire_:%d", circle_suggest_fire_);
//        if(circle_suggest_fire_){
//            track_data.id = 7;
//            circle_suggest_fire_ = false;
//        } else{
//            track_data.id = 0;
//        }
//        track_data.header.frame_id = "base_link";
//        track_data.target_pos.x = min_distance_target_.detections[0].pose.position.x;
//        track_data.target_pos.y = min_distance_target_.detections[0].pose.position.y;
//        track_data.target_pos.z = min_distance_target_.detections[0].pose.position.z;
//        track_data.target_vel.x = 0;
//        track_data.target_vel.y = 0;
//        track_data.target_vel.z = 0;
    }
    else{
        track_data.id = 3;
        track_data.target_pos.x = target_array_.detections[0].pose.position.x;
        track_data.target_pos.y = target_array_.detections[0].pose.position.y;
        track_data.target_pos.z = target_array_.detections[0].pose.position.z;
        track_data.target_vel.x = 0;
        track_data.target_vel.y = 0;
        track_data.target_vel.z = 0;
    }

    track_pub_.publish(track_data);

}

void Forecast_Node::speedCallback(
    const rm_msgs::TargetDetectionArray::Ptr &msg) {
    rm_msgs::TrackData track_data;
    track_data.header.stamp = msg->header.stamp; //??

    if (msg->detections.empty()) {
      track_data.id = 0;
      track_pub_.publish(track_data);
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
    track_data.id = tracker_->tracking_id;
    track_data.target_pos.x = tracker_->target_state(0);
    track_data.target_pos.y = tracker_->target_state(1);
    track_data.target_pos.z = tracker_->target_state(2);
    track_data.target_vel.x = tracker_->target_state(3);
    track_data.target_vel.y = tracker_->target_state(4);
    track_data.target_vel.z = tracker_->target_state(5);
  }

    /***根据观察旋转的结果决定是否建议开火***/
    if (allow_spin_observer_ && spin_observer_) {
//        spin_observer_->max_jump_angle = get_parameter("spin_observer.max_jump_angle").as_double();
//        spin_observer_->max_jump_period = get_parameter("spin_observer.max_jump_period").as_double();
//        spin_observer_->allow_following_range = get_parameter("spin_observer.allow_following_range").as_double();

        spin_observer_->update(track_data, msg->header.stamp, max_jump_angle_, max_jump_period_, allow_following_range_);
//        spin_info_pub_->publish(spin_observer_->spin_info_msg);
    }

  track_pub_.publish(track_data);

    if(!is_reproject_)
        return;
    /***draw reproject***/
    track_data.target_pos.x += track_data.target_vel.x * re_fly_time_;
    track_data.target_pos.y += track_data.target_vel.y * re_fly_time_;
    track_data.target_pos.z += track_data.target_vel.z * re_fly_time_;

    rm_msgs::TargetDetection detection_temp;
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    pose_in.header.frame_id = "odom";
    pose_in.header.frame_id = msg->header.frame_id;
    pose_in.pose.position.x = track_data.target_pos.x;
    pose_in.pose.position.y = track_data.target_pos.y;
    pose_in.pose.position.z = track_data.target_pos.z;

    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                "camera_optical_frame", pose_in.header.frame_id, msg->header.stamp, ros::Duration(1));

        tf2::doTransform(pose_in.pose, pose_out.pose, transform);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
//    ROS_INFO_STREAM(pose_out.pose.position.x
//                    << ",y:" << pose_out.pose.position.y
//                    << ",z:" << pose_out.pose.position.z);

    detection_temp.pose = pose_out.pose;
    Eigen::Vector3d hit_point_cam = {detection_temp.pose.position.x, detection_temp.pose.position.y, detection_temp.pose.position.z};
    target2d_ = reproject(hit_point_cam);
//    std::cout << "target2d = \n" << target2d_ << std::endl;

}

    /**
    * @brief 重投影
    *
    * @param xyz 目标三维坐标
    * @return cv::Point2f 图像坐标系上坐标(x,y)
    */
    cv::Point2f Forecast_Node::reproject(Eigen::Vector3d &xyz)
    {
        Eigen::Matrix3d mat_intrinsic;
        cv2eigen(m_intrinsic_, mat_intrinsic);
        //(u,v,1)^T = (1/Z) * K * (X,Y,Z)^T
        auto result = (1.f / xyz[2]) * mat_intrinsic * (xyz);//解算前进行单位转换
        return cv::Point2f(result[0], result[1]);
    }

    void Forecast_Node::drawCallback(const sensor_msgs::ImageConstPtr& img){
        if(!is_reproject_)
            return;

        cv::Mat origin_img = cv_bridge::toCvShare(img, "bgr8")->image;
        circle(origin_img, target2d_, 10, cv::Scalar(0, 0, 255), -1, 2);
        draw_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", origin_img).toImageMsg());
    }

    template<typename T>
    bool Forecast_Node::initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
    {
        int cnt = 0;
        for(int row = 0;row < matrix.rows();row++)
        {
            for(int col = 0;col < matrix.cols();col++)
            {
                matrix(row,col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

} // namespace rm_forecast

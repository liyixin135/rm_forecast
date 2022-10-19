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
    }

    void Tracker::init(const rm_msgs::TargetDetectionArray & msg)
    {
        if (msg.detections.empty()) {
//            ROS_INFO("no target!");
            return;
        }

        // TODO(chenjun): need more judgement
        // Simply choose the armor that is closest to image center
        // The maximum value that represents a finite floating-point (double) number
        double min_distance = DBL_MAX;
        auto chosen_armor = msg.detections[0];
        /***选择离相机光心距离最近的装甲板***/
        /***replace the distance with confidence***/
        for (const auto & armor : msg.detections) {

            if (armor.confidence < min_distance) {
                min_distance = armor.confidence;
                chosen_armor = armor;
            }
        }

        // KF init
        kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
        Eigen::VectorXd init_state(6);
        const auto position = chosen_armor.pose.position;
        init_state << position.x, position.y, position.z, 0, 0, 0;
        kf_->init(init_state);

        tracking_id = chosen_armor.id;
        tracker_state = DETECTING; /***设置装甲板状态为目标识别中，需要更多帧识别到才开始跟踪***/
    }

    void Tracker::update(const rm_msgs::TargetDetectionArray& msg, const double & dt, const double &max_match_distance, const int &tracking_threshold, const int &lost_threshold)
    {
        // KF predict
        kf_matrices_.F(0, 3) = kf_matrices_.F(1, 4) = kf_matrices_.F(2, 5) = dt; /***更新f矩阵***/
        Eigen::VectorXd kf_prediction = kf_->predict(kf_matrices_.F); /***得出本时刻预测值***/

        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        target_state = kf_prediction; /***本时刻预测值***/

        /***从所有目标中挑选和预测目标离得最近的装甲板***/
        if (!msg.detections.empty()) {
            auto matched_armor = msg.detections[0];
            double min_position_diff = DBL_MAX;
            for (const auto & armor : msg.detections) {
                Eigen::Vector3d position_vec(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
                Eigen::Vector3d predicted_position = kf_prediction.head(3);
                // Difference of the current armor position and tracked armor's predicted position
                double position_diff = (predicted_position - position_vec).norm();
                if (position_diff < min_position_diff) {
                    min_position_diff = position_diff;
                    matched_armor = armor;
                }
            }
            /***如果离预测目标最近的装甲板的距离小于阈值***/
            if (min_position_diff < max_match_distance) {
                // Matching armor found
                matched = true;
                Eigen::Vector3d position_vec(
                        matched_armor.pose.position.x, matched_armor.pose.position.y, matched_armor.pose.position.z);
                /***只有这个时候才做卡尔曼滤波的更新部分***/
                target_state = kf_->update(position_vec);
            }
                /***如果离预测目标最近的装甲板的距离大于阈值，则寻找另一个id相同的装甲板重新作为追踪目标，并把上一个追踪目标的速度给它***/
            else {
                // Check if there is same id armor in current frame 检查当前是否有相同的ID装甲
                for (const auto & armor : msg.detections) {
                    if (armor.id == tracking_id) {
                        matched = true;
                        // Reset KF
                        kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
                        Eigen::VectorXd init_state(6);
                        // Set init state with current armor position and tracking velocity before
                        /***使用新选择的装甲位置和跟踪速度设置初始化状态，***/
                        init_state << armor.pose.position.x, armor.pose.position.y, armor.pose.position.z, tracking_velocity_;
                        kf_->init(init_state);
                        target_state = init_state;
                        break;
                    }
                }
            }
        }

        // Save tracking target velocity 保存跟踪目标速度
        tracking_velocity_ = target_state.tail(3);

        // Tracking state machine
        /***目标识别中，需要更多帧识别到才开始跟踪***/
        if (tracker_state == DETECTING) {
            // DETECTING
            if (matched) {
                detect_count_++;
                if (detect_count_ > tracking_threshold) {
                    detect_count_ = 0;
                    tracker_state = TRACKING;
                }
            } else {
                detect_count_ = 0;
                tracker_state = LOST;
            }

        } else if (tracker_state == TRACKING) {
            // TRACKING
            /***追踪状态时目标丢失则视为暂时消失***/
            if (!matched) {
                tracker_state = TEMP_LOST;
                lost_count_++;
            }

        } else if (tracker_state == TEMP_LOST) {
            // LOST
            if (!matched) {
                lost_count_++;
                if (lost_count_ > lost_threshold) {
                    lost_count_ = 0;
                    tracker_state = LOST;
                }
            }
            else {
                tracker_state = TRACKING;
                lost_count_ = 0;
            }
        }
    }

}  // namespace rm_auto_aim
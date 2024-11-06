#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/affine.hpp>

#include "lab7/srv/hand_eye_calib.hpp"


class HandEyeCalibNode : public rclcpp::Node {

public:
  HandEyeCalibNode();

private:
  void timerCallback();

  void serviceCallback(
    const std::shared_ptr<lab7::srv::HandEyeCalib::Request> request,
    std::shared_ptr<lab7::srv::HandEyeCalib::Response> response);

  void getBase2EndEffectorFrame();
  void getEndEffector2CameraFrame(const aruco_opencv_msgs::msg::ArucoDetection& msg);
  void captureCalibrationMeasure();
  void captureVerificationMeasure();
  void calibrateHandEye();
  void verifyCalibration();
  void broadcastBase2CameraFrame();
  void saveCalibrationOutput();
  void saveVerificationOutput();

  void resetMeasurements();

  std::string createTimeStamp();

  bool is_base2eef_frame_available_ {false};
  bool is_cam2eef_frame_available_ {false};
  bool is_calibration_complete_ {false};
  bool is_verification_complete_ {false};

  int marker_id_ {};
  int measures_captured_quantity_ {};

  std::string robot_base_frame_ {};
  std::string robot_eef_frame_ {};
  std::string workspace_dir_ {};

  geometry_msgs::msg::Transform base2eef_transform_ {};
  geometry_msgs::msg::Pose cam2eef_pose_ {};

  Eigen::Affine3d base2eef_frame_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d cam2eef_frame_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d base2cam_frame_ {Eigen::Affine3d::Identity()};

  // Stores exactly the same matrix of base2cam_frame_ but in cv::Affine3d format.
  cv::Affine3d base2cam_frame_mat_ {cv::Affine3d::Identity()};

  std::vector<cv::Mat> base2eef_frame_tvecs_ {};
  std::vector<cv::Mat> base2eef_frame_rmatxs_ {};
  std::vector<cv::Mat> cam2eef_frame_tvecs_ {};
  std::vector<cv::Mat> cam2eef_frame_rmatxs_ {};

  std::vector<Eigen::Vector3d> estimated_eef_positions_ {};
  std::vector<Eigen::Vector3d> actual_eef_positions_ {};

  std::vector<Eigen::Quaterniond> estimated_eef_orientations_ {};
  std::vector<Eigen::Quaterniond> actual_eef_orientations_ {};

  Eigen::Vector<double, 7> mean_error_vector_ {Eigen::Vector<double, 7>::Zero()};
  Eigen::Vector<double, 7> sum_of_squared_errors_vector_ {Eigen::Vector<double, 7>::Zero()};
  Eigen::Vector<double, 7> root_sum_of_squared_errors_vector_ {Eigen::Vector<double, 7>::Zero()};

  Eigen::Matrix<double, 7, 7> covariance_matrix_ {Eigen::Matrix<double, 7, 7>::Zero()};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ {nullptr};
  geometry_msgs::msg::TransformStamped tf_static_transform_ {};

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};

  rclcpp::TimerBase::SharedPtr timer_ {nullptr};
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr marker_pose_sub_ {nullptr};
  rclcpp::Service<lab7::srv::HandEyeCalib>::SharedPtr hand_eye_calib_service_ {nullptr};

};

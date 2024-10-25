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

  void getBase2GripperFrame();
  void getGripper2CameraFrame(const aruco_opencv_msgs::msg::ArucoDetection& msg);
  void captureMeasure();
  void calibrateHandEye();
  void verifyCalibration();
  void broadcastBase2CameraFrame();
  void saveOutput();

  void resetMeasurements();

  std::string createTimeStamp();

  bool is_base2gripper_frame_available_ {false};
  bool is_cam2gripper_frame_available_ {false};
  bool is_calibration_complete_ {false};

  int marker_id_ {};
  int measures_captured_quantity_ {};

  std::string robot_base_frame_ {"base_link"};
  std::string robot_gripper_frame_ {"wrist_3_link"};
  std::string workspace_dir_ {};

  geometry_msgs::msg::Transform base2gripper_transform_ {};
  geometry_msgs::msg::Pose cam2gripper_pose_ {};

  Eigen::Affine3d base2gripper_frame_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d cam2gripper_frame_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d base2cam_frame_ {Eigen::Affine3d::Identity()};

  // Stores exactly the same matrix of base2cam_frame_ but in cv::Affine3d format.
  cv::Affine3d base2cam_frame_mat_ {cv::Affine3d::Identity()};

  std::vector<cv::Mat> base2gripper_frame_tvecs_ {};
  std::vector<cv::Mat> base2gripper_frame_rmatxs_ {};
  std::vector<cv::Mat> cam2gripper_frame_tvecs_ {};
  std::vector<cv::Mat> cam2gripper_frame_rmatxs_ {};

  std::vector<Eigen::Vector3d> estimated_eef_positions_ {};
  std::vector<Eigen::Vector3d> actual_eef_positions_ {};

  std::vector<Eigen::Quaterniond> estimated_eef_orientations_ {};
  std::vector<Eigen::Quaterniond> actual_eef_orientations_ {};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ {nullptr};
  geometry_msgs::msg::TransformStamped tf_static_transform_ {};

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};

  rclcpp::TimerBase::SharedPtr timer_ {nullptr};
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr marker_pose_sub_ {nullptr};
  rclcpp::Service<lab7::srv::HandEyeCalib>::SharedPtr hand_eye_calib_service_ {nullptr};

};

#pragma once

#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lab7/srv/track_request.hpp"

class EEFPoseTracker : public rclcpp::Node {

public:
  EEFPoseTracker();

private:
  void requestCallback(
    const std::shared_ptr<lab7::srv::TrackRequest::Request> request,
    std::shared_ptr<lab7::srv::TrackRequest::Response> response);

  void timerCallback();
  void plotData();
  std::string createTimeStamp();

  bool should_track_ {false};

  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  std::string from_frame_ {};
  std::string to_frame_ {};
  geometry_msgs::msg::Transform transform_ {};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};

  std::string workspace_dir_ {};
  std::string output_timestamp_ {};
  std::ofstream poses_csv_ {};

  std::string plot_title_ {};

  int8_t plot_axis_x_ {};
  int8_t plot_axis_y_ {};

  std::vector<double> x_points_ {};
  std::vector<double> y_points_ {};
  std::vector<double> z_points_ {};

  rclcpp::Service<lab7::srv::TrackRequest>::SharedPtr track_service_ {nullptr};

};

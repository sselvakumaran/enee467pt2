#pragma once

#include <rclcpp/rclcpp.hpp>

#include "lab7/srv/hand_eye_calib.hpp"

class Lab7CommandInterface : public rclcpp::Node {

public:
  Lab7CommandInterface();
  void sendHandEyeCalibRequest(const lab7::srv::HandEyeCalib::Request::SharedPtr& request);
  void stopNode() {rclcpp::shutdown();}

private:
  bool hand_eye_calib_service_available_ {false};

  rclcpp::Client<lab7::srv::HandEyeCalib>::SharedPtr hand_eye_calib_client_ {nullptr};

};

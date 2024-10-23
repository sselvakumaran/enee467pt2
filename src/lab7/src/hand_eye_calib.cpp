#include <filesystem>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lab7/hand_eye_calib.hpp"

HandEyeCalibNode::HandEyeCalibNode()
: rclcpp::Node("hand_eye_calib_node")
{
  using namespace std::chrono_literals;

  timer_ = this->create_wall_timer(500ms, std::bind(&HandEyeCalibNode::timerCallback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  marker_pose_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "aruco_detections", 10,
    std::bind(&HandEyeCalibNode::getGripper2CameraFrame, this, std::placeholders::_1));

  hand_eye_calib_service_ = this->create_service<lab7::srv::HandEyeCalib>(
    "hand_eye_calib",
    std::bind(&HandEyeCalibNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

  auto param_description {rcl_interfaces::msg::ParameterDescriptor()};

  param_description.description = "Sets the workspace directory for saving the plots and CSV files";
  this->declare_parameter("workspace_dir", "", param_description);

  param_description.description = "ID of the fiducial marker used on the end-effector";
  this->declare_parameter("marker_id", 0, param_description);

  param_description.description = "Name of the robot base frame";
  this->declare_parameter("robot_base_frame", "base_link", param_description);

  param_description.description = "Name of the robot end-effector frame";
  this->declare_parameter("robot_gripper_frame", "wrist_3_link", param_description);

  workspace_dir_ = this->get_parameter("workspace_dir").as_string();

  if (!std::filesystem::is_directory(workspace_dir_)) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "Given workspace directory: " << workspace_dir_ << " does not exist!");

    rclcpp::shutdown();
  }

  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  robot_gripper_frame_ = this->get_parameter("robot_gripper_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration node initialized.");
}

void HandEyeCalibNode::timerCallback()
{
  marker_id_ = this->get_parameter("marker_id").as_int();

  getBase2GripperFrame();
  broadcastBase2CameraFrame();
}

void HandEyeCalibNode::serviceCallback(
  const std::shared_ptr<lab7::srv::HandEyeCalib::Request> request,
  std::shared_ptr<lab7::srv::HandEyeCalib::Response> response)
{
  switch (request->action) {

  case (lab7::srv::HandEyeCalib::Request::CAPTURE):
    captureMeasure();
    break;

  case (lab7::srv::HandEyeCalib::Request::CALIBRATE):
    calibrateHandEye();
    break;

  case (lab7::srv::HandEyeCalib::Request::RESET):
    resetMeasurements();
    break;

  case (lab7::srv::HandEyeCalib::Request::SAVE):
    saveOutput();
    break;

  default:
    response->set__success(false);
    return;

  }

  response->set__success(true);
}

void HandEyeCalibNode::resetMeasurements()
{
  base2gripper_frame_tvecs_.clear();
  base2gripper_frame_rvecs_.clear();
  gripper2cam_frame_tvecs_.clear();
  gripper2cam_frame_rvecs_.clear();

  is_calibration_complete_ = false;

  RCLCPP_INFO(this->get_logger(), "Measurements have been reset, you can start over now.");
}

cv::Vec3d HandEyeCalibNode::rvecFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion_in)
{
  tf2::Quaternion quaternion_tf2 {};
  tf2::convert(quaternion_in, quaternion_tf2);
  quaternion_tf2.normalize();

  double magnitude {2 * std::acos(quaternion_tf2.getW())};
  double sin_half_theta = std::sqrt(1 - std::pow(quaternion_tf2.getW(), 2));

  return {
    magnitude * (quaternion_tf2.getX() / sin_half_theta),
    magnitude * (quaternion_tf2.getY() / sin_half_theta),
    magnitude * (quaternion_tf2.getZ() / sin_half_theta)};
}

void HandEyeCalibNode::getBase2GripperFrame()
{
  try {
    base2gripper_transform_ =
      tf_buffer_->lookupTransform(
        robot_base_frame_, robot_gripper_frame_, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << robot_base_frame_ << " to " << robot_gripper_frame_  << " : "
                            << ex.what());

    is_base2gripper_frame_available_ = false;

    return;
  }

  base2gripper_frame_.translation() = {
    base2gripper_transform_.translation.x,
    base2gripper_transform_.translation.y,
    base2gripper_transform_.translation.z};

  base2gripper_frame_.rvec() = rvecFromQuaternion(base2gripper_transform_.rotation);

  is_base2gripper_frame_available_ = true;
}

void HandEyeCalibNode::getGripper2CameraFrame(const aruco_opencv_msgs::msg::ArucoDetection& msg)
{
  for (const auto& marker_pose: msg.markers) {
    if (marker_pose.marker_id != marker_id_)
      continue;

    gripper2cam_pose_ = marker_pose.pose;

    gripper2cam_frame_.translation() = {
      gripper2cam_pose_.position.x,
      gripper2cam_pose_.position.y,
      gripper2cam_pose_.position.z};

    gripper2cam_frame_.rvec() = rvecFromQuaternion(gripper2cam_pose_.orientation);

    is_gripper2cam_frame_available_ = true;

    return;
  }

  is_gripper2cam_frame_available_ = false;

  return;
}

void HandEyeCalibNode::captureMeasure()
{
  if (!is_base2gripper_frame_available_ || !is_gripper2cam_frame_available_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Measure capture failed: One/Both of the frames is unavailable, try again.");

    return;
  }

  base2gripper_frame_tvecs_.push_back(base2gripper_frame_.translation());
  base2gripper_frame_rvecs_.push_back(base2gripper_frame_.rvec());

  gripper2cam_frame_tvecs_.push_back(gripper2cam_frame_.translation());
  gripper2cam_frame_rvecs_.push_back(gripper2cam_frame_.rvec());

  measures_captured_quantity_++;

  RCLCPP_INFO(this->get_logger(), "Measure captured successfully.");
  RCLCPP_INFO_STREAM(this->get_logger(), "Measures captured: " << measures_captured_quantity_);
}

void HandEyeCalibNode::calibrateHandEye()
{
  if (measures_captured_quantity_ < 3) {
    RCLCPP_WARN(this->get_logger(), "Calibration failed: Insufficient measurements.");
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Get " << 3 - measures_captured_quantity_ << " more measurements and try again.");

    is_calibration_complete_ = false;

    return;
  }

  // cv::Mat3d base2cam_rotation_matrix {};
  // cv::Vec3d base2cam_translation_vector {};

  RCLCPP_INFO(this->get_logger(), "Calibrating... This might take some while.");

  cv::calibrateRobotWorldHandEye(
    gripper2cam_frame_rvecs_, gripper2cam_frame_tvecs_,
    base2gripper_frame_rvecs_, base2gripper_frame_tvecs_,
    base2cam_frame_.rotation(), base2cam_frame_.translation(),
    cv::noArray(), cv::noArray());

  is_calibration_complete_ = true;
  RCLCPP_INFO(this->get_logger(), "Hand-eye calibration compelte!");
  RCLCPP_INFO(this->get_logger(), "Estimated frame will now be broadcasted.");
}

void HandEyeCalibNode::broadcastBase2CameraFrame()
{
  if (!is_calibration_complete_)
    return;

  tf_static_transform_.header.stamp = this->get_clock()->now();
  tf_static_transform_.header.frame_id = robot_base_frame_;
  tf_static_transform_.child_frame_id = "test_camera_frame";

  tf_static_transform_.transform.translation.x = base2cam_frame_.translation()[0];
  tf_static_transform_.transform.translation.y = base2cam_frame_.translation()[1];
  tf_static_transform_.transform.translation.z = base2cam_frame_.translation()[2];

  tf2::Quaternion rotation_quaternion;

  tf2::Vector3 rotation_vector {};
  rotation_vector.setX(base2cam_frame_.rvec()[0]);
  rotation_vector.setY(base2cam_frame_.rvec()[1]);
  rotation_vector.setZ(base2cam_frame_.rvec()[2]);

  double rotation_angle {
    std::sqrt(
        std::pow(base2cam_frame_.rvec()[0], 2)
      + std::pow(base2cam_frame_.rvec()[1], 2)
      + std::pow(base2cam_frame_.rvec()[2], 2))};

  rotation_quaternion.setRotation(rotation_vector, rotation_angle);

  tf_static_broadcaster_->sendTransform(tf_static_transform_);
}

std::string HandEyeCalibNode::createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  return timeStamp.str();
}

void HandEyeCalibNode::saveOutput()
{
  if (!is_calibration_complete_) {
    RCLCPP_WARN(this->get_logger(), "Calibration is incomplete, not saving the file.");

    return;
  }

  if (!std::filesystem::is_directory(workspace_dir_ + "/output/lab7"))
    std::filesystem::create_directories(workspace_dir_ + "/output/lab7");

  std::string output_file_name {
    workspace_dir_ + "/output/lab7/frame-" + createTimeStamp() + ".yaml"};

  cv::FileStorage output_file {output_file_name, cv::FileStorage::WRITE};

  if (!output_file.isOpened()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");

    return;
  }

  output_file.writeComment("\nTransformation from base to camera frame");
  output_file << base2cam_frame_.matrix;
  output_file.release();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto hand_eye_calib_node {std::make_shared<HandEyeCalibNode>()};

  rclcpp::spin(hand_eye_calib_node);

  rclcpp::shutdown();

  return 0;
}

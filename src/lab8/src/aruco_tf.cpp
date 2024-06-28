#include "../include/aruco_tf.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
// Test Node
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      this->declare_parameter("mine", "hello");

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

      subscriber_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("aruco_detections", 10, std::bind(&MinimalPublisher::callback, this, std::placeholders::_1));
    }

  private:
    void callback(const aruco_opencv_msgs::msg::ArucoDetection & msg) const {
      aruco_opencv_msgs::msg::ArucoDetection detect = msg;
      RCLCPP_INFO(this->get_logger(), "Hmmm");
    }
    void timer_callback()
    {
      std::string param = this->get_parameter("mine").as_string();

      auto message = std_msgs::msg::String();
      message.data = ", world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s%s'", param.c_str(), message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr subscriber_;
    aruco_opencv_msgs::msg::ArucoDetection aruco_detection;
    };



// ArucoTF Functions
/**
 * @brief Function to save calibration data to file
 * 
 * @param save_rot Rotation extrinsic
 * @param save_trans Translation extrinsic
 */
void ArucoTF::saveCalibToFile(const Eigen::Quaternionf &save_rot, const Eigen::Vector3f &save_trans){
  if (!ArucoTF::calib){
    RCLCPP_INFO(this->get_logger(), "Saving calibration to file");

    std::string calib_path = ament_index_cpp::get_package_share_directory("lab8");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    auto index = calib_path.find("install");
    auto actual_path = calib_path.substr(0, index);
    actual_path += "src/lab8/calibration/camera/logitech_extrinsics.json";

    std::vector<float> rot, trans;

    // Convert quaternion (w,x,y,z) from Eigen to Vector
    rot.push_back(save_rot.w());
    rot.push_back(save_rot.x());
    rot.push_back(save_rot.y());
    rot.push_back(save_rot.z());

    // Convert translation from Eigen to Vector
    trans.push_back(save_trans(0));
    trans.push_back(save_trans(1));
    trans.push_back(save_trans(2));

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      RCLCPP_WARN(this->get_logger(), "JSON Parse failed: %s", e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
    }

    // Add rotation and translation to json object
    calib_data[calib_section]["rot"] = rot;
    calib_data[calib_section]["trans"] = trans;

    std::ofstream calib_file_out(calib_path,
                                 std::fstream::out | std::ofstream::trunc);
    calib_file_out << std::setprecision(16) << calib_data;
    calib_file_out.close();

    std::ofstream actual_file_out(actual_path,
                                 std::fstream::out | std::ofstream::trunc);
    actual_file_out << std::setprecision(16) << calib_data;
    actual_file_out.close();
  }
}


/**
 * @brief Function to load calibration data from file
 * 
 */
void ArucoTF::loadCalibFromFile() {
  if (!ArucoTF::calib) {
    RCLCPP_INFO(this->get_logger(), "Saving calibration to file");

    std::string calib_path = ament_index_cpp::get_package_share_directory("lab8");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json objectl
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      RCLCPP_WARN(this->get_logger(), "JSON Parse failed: %s", e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
      // Get translation and rotation data from json
      std::vector<float> trans_json = calib_data[calib_section]["trans"];
      std::vector<float> rot_json = calib_data[calib_section]["rot"];

      // Vector in (x, y, z) format
      Eigen::Vector3f trans;
      trans(0) = trans_json[0];
      trans(1) = trans_json[1];
      trans(2) = trans_json[2];
      // Quaternion in (w,x,y,z) format
      Eigen::Quaternionf quat;
      quat.w() = rot_json[0];
      quat.x() = rot_json[1];
      quat.y() = rot_json[2];
      quat.z() = rot_json[3];

      std::cout << "Translation: " << trans_json << std::endl;
      std::cout << "Rotation: " << rot_json << std::endl;

      // Apply calibration data to camera
      tf2::Quaternion rot_quat_camToWorld(quat.x(), quat.y(), quat.z(), quat.w());
      rot_quat_camToWorld.normalize();

      // Get translation in tf2
      tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

      // Convert to tf2
      ArucoTF::tf_camToWorld.setRotation(rot_quat_camToWorld);
      ArucoTF::tf_camToWorld.setOrigin(trans_camToWorld);
      
      // Set calibrated
      ArucoTF::calib = true;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
      ArucoTF::calib = false;
    }
  }
}

// Remove setTFCamToWorld Fn

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
  // Initialise ROS
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(10.0);

  auto calibrate_cam = std::make_shared<ArucoTF>();

  RCLCPP_INFO(calibrate_cam->get_logger(), "------------------------------------------------------");
  
  const Eigen::Vector3f a(1, 2, 3);
  const Eigen::Quaternionf b(1, 2, 3, 4);

  // Testing Function
  calibrate_cam->loadCalibFromFile();


  rclcpp::spin(std::make_shared<ArucoTF>());
  tf2::Transform tf_MarkerToWorld;
  geometry_msgs::msg::Pose marker_pose;

  // Do while ok() 


  // Testing Eigen variables
  // std::cout << calibrate_cam->samples_camToMarker.rows() << " " << calibrate_cam->samples_camToMarker.cols() << "\n";
  // std::cout << calibrate_cam->samples_markerToWorld.rows() << " " << calibrate_cam->samples_markerToWorld.cols() << "\n";

  // calibrate_cam->samples_markerToWorld.col(0) = Eigen::Vector3f(1, 2, 3).transpose();
  // calibrate_cam->samples_camToMarker.col(0) = Eigen::Vector3f(4, 5, 3).transpose();

  // std::cout << calibrate_cam->samples_markerToWorld.col(0);
  // std::cout << calibrate_cam->samples_camToMarker.col(0);


  // This works for query one message
  // auto message = std_msgs::msg::String();
  // bool found = rclcpp::wait_for_message(message, calibrate_cam, "chatter", std::chrono::seconds(1));
  // RCLCPP_INFO(calibrate_cam->get_logger(), "scan found= %d", found);
  // RCLCPP_INFO(calibrate_cam->get_logger(), "Text found: %s", message.data.c_str());


  // End execution
  rclcpp::shutdown();
  return 0;
}

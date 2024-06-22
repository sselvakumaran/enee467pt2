#include "../include/aruco_tf.hpp"
#include <rclcpp/rate.hpp>

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






int main(int argc, char * argv[])
{
  // Initialise ROS
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(10.0);

  auto calibrate_cam = std::make_shared<ArucoTF>();

  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::spin(calibrate_cam);

//consider do i need to spin? or can i just call the fns?
//i require the info from topic /aruco_detections
// test wait_for_message using *this

  // End execution
  rclcpp::shutdown();
  return 0;
}
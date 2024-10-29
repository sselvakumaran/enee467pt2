#include "lab7/command_interface.hpp"

Lab7CommandInterface::Lab7CommandInterface()
: rclcpp::Node("command_interface")
{
  hand_eye_calib_client_ = this->create_client<lab7::srv::HandEyeCalib>("hand_eye_calib");

  hand_eye_calib_service_available_ =
    hand_eye_calib_client_->wait_for_service(std::chrono::seconds(3));
}

void Lab7CommandInterface::sendHandEyeCalibRequest(
  const lab7::srv::HandEyeCalib::Request::SharedPtr& request)
{
  if (!hand_eye_calib_service_available_) {
    RCLCPP_WARN(this->get_logger(), "Request failed because the service is unavailable.");

    return;
  }

  auto result_future {hand_eye_calib_client_->async_send_request(request).share()};

  RCLCPP_INFO(this->get_logger(), "Command sent, waiting for result...");
  auto result_status {result_future.wait_for(std::chrono::seconds(1))};

  if (result_future.get()->success && result_status == std::future_status::ready)
    RCLCPP_INFO(this->get_logger(), "Command request fulfilled!");
  else
    RCLCPP_INFO(this->get_logger(), "Command request failed to execute! Try again.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto command_interface_node {std::make_shared<Lab7CommandInterface>()};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(command_interface_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::cout << "\nFor Hand-eye calibration, use the following commands:\n"
            << "  Press ENTER to capture a measurement\n"
            << "  Type 'calibrate' and press ENTER to perform the hand-eye calibration\n"
            << "  Type 'verify' and press ENTER to verify calibration result\n"
            << "  Type 'reset' and press ENTER to clear the measurements and start over\n"
            << "  Type 'save' and press ENTER to save the calibration/verification result\n"
            << "  Type 'exit' and press ENTER to exit\n";

  std::string input {};
  auto request {std::make_shared<lab7::srv::HandEyeCalib::Request>()};

  do {
    std::cout << "\nYour command: ";
    std::getline(std::cin, input);

    if (input.empty()) {
      request->set__action(lab7::srv::HandEyeCalib::Request::CAPTURE);
      command_interface_node->sendHandEyeCalibRequest(request);
    }

    else if (input == "calibrate") {
      request->set__action(lab7::srv::HandEyeCalib::Request::CALIBRATE);
      command_interface_node->sendHandEyeCalibRequest(request);
    }

    else if (input == "verify") {
      request->set__action(lab7::srv::HandEyeCalib::Request::VERIFY);
      command_interface_node->sendHandEyeCalibRequest(request);
    }

    else if (input == "reset") {
      request->set__action(lab7::srv::HandEyeCalib::Request::RESET);
      command_interface_node->sendHandEyeCalibRequest(request);
    }

    else if (input == "save") {
      request->set__action(lab7::srv::HandEyeCalib::Request::SAVE);
      command_interface_node->sendHandEyeCalibRequest(request);
    }

    else if (input != "exit") {
      std::cerr << "Please provide a correct input and try again." << '\n';
    }


  } while (input != "exit");

  command_interface_node->stopNode();

  return 0;
}

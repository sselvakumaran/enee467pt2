#include "lab7/ur3e_move_interface.hpp"

int main(int argc, char** argv)
{
  std::string shape_argument {};
  std::string orientation_argument {};
  std::string size_argument {};

  if (argv[1])
    shape_argument = argv[1];

  if (argv[2])
    orientation_argument = argv[2];

  if (argv[3])
    size_argument = argv[3];

  double size {};
  try {
    size = std::stod(size_argument);
  }
  catch (std::exception& exception) {
    std::cout << '\n' << "A valid size value isn't given, default will be used instead." << '\n'
              << '\n';
    size = 0;
  }

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto mover_node {std::make_shared<UR3eMoveInterface>(node_options)};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mover_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  if (shape_argument == "circle" && orientation_argument == "horizontal")
    mover_node->drawCircleXY(size);

  else if (shape_argument == "circle" && orientation_argument == "vertical")
    mover_node->drawCircleYZ(size);

  else if (shape_argument == "square" && orientation_argument == "horizontal")
    mover_node->drawSquareXY(size);

  else if (shape_argument == "square" && orientation_argument == "vertical")
    mover_node->drawSquareYZ(size);

  else
    mover_node->examplesMoveIt();

  rclcpp::shutdown();

  return 0;
}

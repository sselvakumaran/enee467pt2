#include "lab8/ur3e_move_interface.hpp"

void UR3eMoveInterface::drawCircleXY(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  std::vector<double> target_joint_positions {M_PI_2, -M_PI_4, M_PI_2, -3*M_PI_4, -M_PI_2, 0};

  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto current_pose = move_group_interface_->getCurrentPose().pose;
  waypoints.push_back(current_pose);

  int RESOLUTION = 360;
  double DELTA_THETA = 2 * M_PI / RESOLUTION;
  auto center = current_pose;
  center.position.y -= radius_meters;

  // auto new_pose = center;
  // new_pose.position.y -= 0.2;
  // waypoints.push_back(new_pose);

  for (int i = 0; i <= RESOLUTION + 10; i++) {
    auto new_pose = center;
    auto theta = M_PI_2 + DELTA_THETA * (double) i;
    new_pose.position.x += radius_meters*cos(theta);
    new_pose.position.y += radius_meters*sin(theta);
    waypoints.push_back(new_pose);
  }

  RCLCPP_WARN(this->get_logger(), ".");

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Circle in the Horizontal plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::X_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Y_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  // Step 6
  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target

  move_group_interface_->setNamedTarget("home");
  move_group_interface_->move();
}

void UR3eMoveInterface::drawCircleYZ(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  std::vector<double> target_joint_positions {-M_PI_2,
    -(9.0/8.0) * M_PI_2, -(3.0/8.0) * M_PI_2,
    -(1.0/2.0) * M_PI_2, M_PI_2, 0};

  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto current_pose = move_group_interface_->getCurrentPose().pose;
  waypoints.push_back(current_pose);

  int RESOLUTION = 20;
  double DELTA_THETA = 2 * M_PI / RESOLUTION;
  auto center = current_pose;
  center.position.z -= radius_meters;

  for (int i = 0; i <= RESOLUTION + 10; i++) {
    auto new_pose = center;
    auto theta = M_PI_2 + DELTA_THETA * (double) i;
    new_pose.position.y += radius_meters*cos(theta);
    new_pose.position.z += radius_meters*sin(theta);
    waypoints.push_back(new_pose);
  }

  RCLCPP_WARN(this->get_logger(), ".");

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Circle in the Vertical plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::Y_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Z_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target

  move_group_interface_->setNamedTarget("home");
  move_group_interface_->move();
}

void UR3eMoveInterface::drawSquareXY(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) / sqrt(2);

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  std::vector<double> target_joint_positions {M_PI_2, -M_PI_4, M_PI_2, -3*M_PI_4, -M_PI_2, 0};

  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);
  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto current_pose = move_group_interface_->getCurrentPose().pose;
  waypoints.push_back(current_pose);

  auto pose1 = current_pose;
  pose1.position.x += side_meters;
  waypoints.push_back(pose1);

  auto pose2 = pose1;
  pose2.position.y -= side_meters;
  waypoints.push_back(pose2);

  auto pose3 = pose2;
  pose3.position.x -= side_meters;
  waypoints.push_back(pose3);

  auto pose4 = pose3;
  pose4.position.y += side_meters;
  waypoints.push_back(pose4);

  // Step 4
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

  // Step 5
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  // Additional Step: Create and send a track request to start tracking the end-effector pose
  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Square in the horizontal plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::X_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Y_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  // Step 6
  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  // Stop tracking the end effector by sending a STOP request.
  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target
  move_group_interface_->setNamedTarget("home");
  move_group_interface_->move();
}

void UR3eMoveInterface::drawSquareYZ(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) / sqrt(2);

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  std::vector<double> target_joint_positions {-M_PI_2, -M_PI_2, -M_PI_2, 0, M_PI_2, 0};

  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);
  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto current_pose = move_group_interface_->getCurrentPose().pose;
  waypoints.push_back(current_pose);

  auto pose1 = current_pose;
  pose1.position.y += side_meters;
  waypoints.push_back(pose1);

  auto pose2 = pose1;
  pose2.position.z -= side_meters;
  waypoints.push_back(pose2);

  auto pose3 = pose2;
  pose3.position.y -= side_meters;
  waypoints.push_back(pose3);

  auto pose4 = pose3;
  pose4.position.z += side_meters;
  waypoints.push_back(pose4);

  // Step 4
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

  // Step 5
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  // Additional Step: Create and send a track request to start tracking the end-effector pose
  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Square in the vertical plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::Y_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Z_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  // Step 6
  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  // Stop tracking the end effector by sending a STOP request.
  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target
  move_group_interface_->setNamedTarget("home");
  move_group_interface_->move();
}

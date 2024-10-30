#include "lab8/ur3e_move_interface.hpp"

void UR3eMoveInterface::drawCircleXY(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};

  move_group_interface_->setJointValueTarget(target_joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;

  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto current_pose = move_group_interface_->getCurrentPose().pose;
  waypoints.push_back(current_pose);

  int RESOLUTION = 10;
  double DELTA_THETA = M_PI / RESOLUTION;
  auto center = current_pose;
  center.position.y -= radius_meters;
  for (int i = 0; i < RESOLUTION; i++) {
    auto new_pose = center;
    auto theta = M_PI_2 + DELTA_THETA * i;
    new_pose.position.x += radius_meters*cos(theta);
    new_pose.position.y += radius_meters*sin(theta);
    waypoints.push_back(new_pose);
  }

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};
  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

    track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "Example: A Circle in the Horizontal plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::X_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Y_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  // Step 6
  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  /// TODO: Move the robot back to its home position by setting a named target

  move_group_interface_->setNamedTarget("home");
  move_group_interface_->move();

  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);
}

void UR3eMoveInterface::drawCircleYZ(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

void UR3eMoveInterface::drawSquareXY(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) / sqrt(2);

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

void UR3eMoveInterface::drawSquareYZ(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) / sqrt(2);

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

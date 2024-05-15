#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class UR3eMover : public rclcpp::Node
{
  public:
    explicit UR3eMover(const std::string& planning_group_name);

    void examplesMoveIt();

    void drawCircleXY(double radius_meters = 0);
    void drawCircleYZ(double radius_meters = 0);

    void drawSquareXY(double side_meters = 0);
    void drawSquareYZ(double side_meters = 0);

  private:
    void planToJointSpaceGoal(
      const std::vector<double>& target_joint_positions,
      moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

    void planCartesianPath(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      moveit_msgs::msg::RobotTrajectory& trajectory);

    void planToPoseGoal(
      const geometry_msgs::msg::Pose& pose_goal,
      moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

    moveit::planning_interface::MoveGroupInterface move_group_;
};

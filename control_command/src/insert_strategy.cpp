#include "insert_strategy.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool InsertStrategy::execute(moveit::planning_interface::MoveGroupInterface &move_group,
                             rclcpp::Logger logger,
                             double insert_distance,
                             double percent,
                             const std::string &mode)
{
  RCLCPP_INFO(logger, "[InsertStrategy] Starting insertion: dz=%.4f m, speed=%.1f%%, mode=%s",
              insert_distance, percent, mode.c_str());

  // --- 设置速度缩放 ---
  double scale = std::min(1.0, std::max(0.01, percent / 100.0));
  move_group.setMaxVelocityScalingFactor(scale);
  move_group.setMaxAccelerationScalingFactor(scale);

  // --- 获取当前姿态 ---
  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
  geometry_msgs::msg::Pose target_pose = current_pose.pose;

  // --- 根据模式计算目标 ---
  if (mode == "joint")
  {
    // 在末端执行器(J6)局部Z方向插补
    tf2::Transform tf_eef;
    tf2::fromMsg(current_pose.pose, tf_eef);

    tf2::Vector3 translation(0, 0, insert_distance);  // 局部Z
    tf2::Transform tf_translation;
    tf_translation.setIdentity();
    tf_translation.setOrigin(translation);

    tf2::Transform tf_target = tf_eef * tf_translation;
    tf2::toMsg(tf_target, target_pose);
    RCLCPP_INFO(logger, "[InsertStrategy] Target pose computed along J6 local Z");
  }
  else if (mode == "world")
  {
    // 世界坐标系 Z 方向插补
    target_pose.position.z += insert_distance;
    RCLCPP_INFO(logger, "[InsertStrategy] Target pose computed along world Z");
  }
  else
  {
    RCLCPP_WARN(logger, "[InsertStrategy] Unknown mode '%s', fallback to world Z", mode.c_str());
    target_pose.position.z += insert_distance;
  }

  // --- 规划路径 ---
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

  RCLCPP_INFO(logger, "[InsertStrategy] Path computation result: %.1f%%", fraction * 100);

  if (fraction > 0.9)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_res = move_group.execute(plan);

    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "[InsertStrategy] Insertion executed successfully!");
      return true;
    }
    else
    {
      RCLCPP_ERROR(logger, "[InsertStrategy] Motion execution failed!");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(logger, "[InsertStrategy] Only %.1f%% of path planned (<90%%), aborting.",
                fraction * 100);
    return false;
  }
}

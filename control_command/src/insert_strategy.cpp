#include "insert_strategy.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>

// MoveIt2 Humble headers
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// ========== 工具函数 ==========

// 计算四元数夹角（0~pi）
static double quatAngularDistance(const Eigen::Quaterniond& a,
                                  const Eigen::Quaterniond& b)
{
  double dot = std::abs(a.dot(b));
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot);
}

// 轨迹直线性/姿态变化验收（可选）
static bool verifyLinearInsertion(const robot_trajectory::RobotTrajectory& rt,
                                  const std::string& eef_link,
                                  const Eigen::Vector3d& p0,
                                  const Eigen::Vector3d& p1,
                                  const Eigen::Quaterniond& q0,
                                  bool check_orientation,
                                  double pos_tol,   // 位置横向误差阈值（m）
                                  double ang_tol)   // 姿态变化阈值（rad）
{
  const Eigen::Vector3d dir = (p1 - p0).normalized();
  const double length = (p1 - p0).norm();
  if (length <= 1e-6) return false;

  double max_lateral_err = 0.0;
  double max_ang_err = 0.0;

  const std::size_t N = rt.getWayPointCount();
  for (std::size_t i = 0; i < N; ++i)
  {
    const moveit::core::RobotState& st = rt.getWayPoint(i);
    const Eigen::Isometry3d T = st.getGlobalLinkTransform(eef_link);

    const Eigen::Vector3d p = T.translation();
    const Eigen::Quaterniond q(T.rotation());

    // 位置横向误差
    const Eigen::Vector3d v = p - p0;
    const double s = v.dot(dir);
    const Eigen::Vector3d p_on_line = p0 + std::min(std::max(0.0, s), length) * dir;
    const double lateral_err = (p - p_on_line).norm();
    if (lateral_err > max_lateral_err) max_lateral_err = lateral_err;

    // 姿态误差
    if (check_orientation)
    {
      const double ang = quatAngularDistance(q0, q);
      if (ang > max_ang_err) max_ang_err = ang;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("InsertStrategy"),
              "[Verify] max lateral err = %.3f mm, max ang err = %.2f deg",
              max_lateral_err * 1000.0, max_ang_err * 180.0 / M_PI);

  if (max_lateral_err > pos_tol) return false;
  if (check_orientation && max_ang_err > ang_tol) return false;
  return true;
}

// 规范化 mode 字符串：trim + tolower
static std::string normalize_mode(std::string s)
{
  auto not_space = [](int ch){ return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// ========== 主实现 ==========

bool InsertStrategy::execute(moveit::planning_interface::MoveGroupInterface &move_group,
                             rclcpp::Logger logger,
                             double insert_distance,
                             double percent,
                             const std::string &mode_in)
{
  const std::string mode = normalize_mode(mode_in);
  RCLCPP_INFO(logger, "[InsertStrategy] dz=%.4f m, speed=%.1f%%, mode=%s",
              insert_distance, percent, mode.c_str());

  // 速度缩放
  const double scale = std::min(1.0, std::max(0.01, percent / 100.0));
  move_group.setMaxVelocityScalingFactor(scale);
  move_group.setMaxAccelerationScalingFactor(scale);

  // 统一 tip link
  const std::string eef = move_group.getEndEffectorLink().empty()
                            ? std::string("J6")
                            : move_group.getEndEffectorLink();
  move_group.setEndEffectorLink(eef);

  // 当前姿态 / 参考系
  geometry_msgs::msg::PoseStamped current_ps = move_group.getCurrentPose(eef);
  move_group.setPoseReferenceFrame(current_ps.header.frame_id);
  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose = current_ps.pose;

  // 规划参数（与旧代码一致的行为）
  const double eef_step_param = 0.005;   // 5 mm 插值步长
  const double jump_threshold  = 0.0;    // 关闭关节跳变检测（与旧代码一致）
  const bool   avoid_collisions = false; // 与旧代码一致；上机请改回 true

  // 生成 waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.clear();

  if (mode == "joint")
  {
    // 以当前 EEF 坐标系为基，沿其 Z 轴离散生成路径（每个点都用 tf_eef * translation）
    tf2::Transform tf_eef;
    tf2::fromMsg(current_ps.pose, tf_eef);

    const double total = insert_distance;
    const int steps = std::max(2, static_cast<int>(std::ceil(std::abs(total) / eef_step_param)) + 1);

    for (int i = 1; i <= steps; ++i) {
      double alpha = static_cast<double>(i) / steps;
#include "insert_strategy.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>

// MoveIt2 Humble headers
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// ========== 工具函数 ==========

// 计算四元数夹角（0~pi）
static double quatAngularDistance(const Eigen::Quaterniond& a,
                                  const Eigen::Quaterniond& b)
{
  double dot = std::abs(a.dot(b));
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot);
}

// 轨迹直线性/姿态变化验收（可选）
static bool verifyLinearInsertion(const robot_trajectory::RobotTrajectory& rt,
                                  const std::string& eef_link,
                                  const Eigen::Vector3d& p0,
                                  const Eigen::Vector3d& p1,
                                  const Eigen::Quaterniond& q0,
                                  bool check_orientation,
                                  double pos_tol,   // 位置横向误差阈值（m）
                                  double ang_tol)   // 姿态变化阈值（rad）
{
  const Eigen::Vector3d dir = (p1 - p0).normalized();
  const double length = (p1 - p0).norm();
  if (length <= 1e-6) return false;

  double max_lateral_err = 0.0;
  double max_ang_err = 0.0;

  const std::size_t N = rt.getWayPointCount();
  for (std::size_t i = 0; i < N; ++i)
  {
    const moveit::core::RobotState& st = rt.getWayPoint(i);
    const Eigen::Isometry3d T = st.getGlobalLinkTransform(eef_link);

    const Eigen::Vector3d p = T.translation();
    const Eigen::Quaterniond q(T.rotation());

    // 位置横向误差
    const Eigen::Vector3d v = p - p0;
    const double s = v.dot(dir);
    const Eigen::Vector3d p_on_line = p0 + std::min(std::max(0.0, s), length) * dir;
    const double lateral_err = (p - p_on_line).norm();
    if (lateral_err > max_lateral_err) max_lateral_err = lateral_err;

    // 姿态误差
    if (check_orientation)
    {
      const double ang = quatAngularDistance(q0, q);
      if (ang > max_ang_err) max_ang_err = ang;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("InsertStrategy"),
              "[Verify] max lateral err = %.3f mm, max ang err = %.2f deg",
              max_lateral_err * 1000.0, max_ang_err * 180.0 / M_PI);

  if (max_lateral_err > pos_tol) return false;
  if (check_orientation && max_ang_err > ang_tol) return false;
  return true;
}

// 规范化 mode 字符串：trim + tolower
static std::string normalize_mode(std::string s)
{
  auto not_space = [](int ch){ return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// ========== 主实现 ==========

bool InsertStrategy::execute(moveit::planning_interface::MoveGroupInterface &move_group,
                             rclcpp::Logger logger,
                             double insert_distance,
                             double percent,
                             const std::string &mode_in)
{
  const std::string mode = normalize_mode(mode_in);
  RCLCPP_INFO(logger, "[InsertStrategy] dz=%.4f m, speed=%.1f%%, mode=%s",
              insert_distance, percent, mode.c_str());

  // 速度缩放
  const double scale = std::min(1.0, std::max(0.01, percent / 100.0));
  move_group.setMaxVelocityScalingFactor(scale);
  move_group.setMaxAccelerationScalingFactor(scale);

  // 统一 tip link
  const std::string eef = move_group.getEndEffectorLink().empty()
                            ? std::string("J6")
                            : move_group.getEndEffectorLink();
  move_group.setEndEffectorLink(eef);

  // 当前姿态 / 参考系
  geometry_msgs::msg::PoseStamped current_ps = move_group.getCurrentPose(eef);
  move_group.setPoseReferenceFrame(current_ps.header.frame_id);
  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose = current_ps.pose;

  // 规划参数（与旧代码一致的行为）
  const double eef_step_param = 0.005;   // 5 mm 插值步长
  const double jump_threshold  = 0.0;    // 关闭关节跳变检测（与旧代码一致）
  const bool   avoid_collisions = false; // 与旧代码一致；上机请改回 true

  // 生成 waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.clear();

  if (mode == "joint")
  {
    // 以当前 EEF 坐标系为基，沿其 Z 轴离散生成路径（每个点都用 tf_eef * translation）
    tf2::Transform tf_eef;
    tf2::fromMsg(current_ps.pose, tf_eef);

    const double total = insert_distance;
    const int steps = std::max(2, static_cast<int>(std::ceil(std::abs(total) / eef_step_param)) + 1);

    for (int i = 1; i <= steps; ++i) {
      double alpha = static_cast<double>(i) / steps;

      tf2::Transform tf_translation;
      tf_translation.setIdentity();
      tf_translation.setOrigin(tf2::Vector3(0, 0, alpha * total)); // **局部 Z**

      tf2::Transform tf_target = tf_eef * tf_translation;

      geometry_msgs::msg::Pose pose_i;
      tf2::toMsg(tf_target, pose_i);
      // 也可强制锁姿态为初始姿态（两者等价）：
      // pose_i.orientation = current_ps.pose.orientation;

      waypoints.push_back(pose_i);
    }

    // 打印“局部 Z 在世界的投影”，仅用于确认方向
    Eigen::Quaterniond q_we(current_ps.pose.orientation.w,
                            current_ps.pose.orientation.x,
                            current_ps.pose.orientation.y,
                            current_ps.pose.orientation.z);
    Eigen::Vector3d z_w = q_we.toRotationMatrix().col(2);
    RCLCPP_INFO(logger, "[InsertStrategy] Tool Z in world ~= [%.3f %.3f %.3f], steps=%zu, ref=%s, tip=%s",
                z_w.x(), z_w.y(), z_w.z(),
                waypoints.size(), current_ps.header.frame_id.c_str(), eef.c_str());
  }
  else if (mode == "world")
  {
    // 按世界 Z 离散（保持行为对称）
    const double total = insert_distance;
    const int steps = std::max(2, static_cast<int>(std::ceil(std::abs(total) / eef_step_param)) + 1);
    for (int i = 1; i <= steps; ++i) {
      double alpha = static_cast<double>(i) / steps;
      geometry_msgs::msg::Pose pose_i = current_ps.pose;
      pose_i.position.z += alpha * total;
      waypoints.push_back(pose_i);
    }
  }
  else
  {
    RCLCPP_WARN(logger, "[InsertStrategy] Unknown mode '%s', abort.", mode_in.c_str());
    return false; // 不再 fallback 到 world Z
  }

  // 规划（不再回退世界 Z）
  moveit_msgs::msg::RobotTrajectory traj_msg;
  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step_param, jump_threshold, traj_msg, avoid_collisions);

  RCLCPP_INFO(logger, "[InsertStrategy] Cartesian path fraction: %.1f%%", fraction * 100.0);
  if (fraction < 0.9) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Only %.1f%% of path planned (<90%%). No fallback. Abort.",
                 fraction * 100.0);
    return false;
  }

  // 可选：验收“直线 & 姿态”
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Failed to get current state for verification.");
    return false;
  }
  moveit::core::RobotModelConstPtr model = current_state->getRobotModel();

  robot_trajectory::RobotTrajectory rt(model, move_group.getName());
  rt.setRobotTrajectoryMsg(*current_state, traj_msg);

  const Eigen::Vector3d p0(current_ps.pose.position.x,
                           current_ps.pose.position.y,
                           current_ps.pose.position.z);
  const Eigen::Vector3d p1(waypoints.back().position.x,
                           waypoints.back().position.y,
                           waypoints.back().position.z);
  const Eigen::Quaterniond q0(current_ps.pose.orientation.w,
                              current_ps.pose.orientation.x,
                              current_ps.pose.orientation.y,
                              current_ps.pose.orientation.z);

  const bool keep_orientation = (mode == "joint");
  const double pos_tol = 0.002;          // 2 mm
  const double ang_tol = 5.0 * M_PI/180; // 5 度
  if (!verifyLinearInsertion(rt, eef, p0, p1, q0, keep_orientation, pos_tol, ang_tol)) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Verification failed. Abort.");
    return false;
  }

  // 时间参数化 & 执行
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, scale, scale)) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Time parameterization failed.");
    return false;
  }
  rt.getRobotTrajectoryMsg(traj_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj_msg;

  auto exec_res = move_group.execute(plan);
  if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "[InsertStrategy] Insertion executed successfully!");
    return true;
  }
  RCLCPP_ERROR(logger, "[InsertStrategy] Motion execution failed!");
  return false;
}

      tf2::Transform tf_translation;
      tf_translation.setIdentity();
      tf_translation.setOrigin(tf2::Vector3(0, 0, alpha * total)); // **局部 Z**

      tf2::Transform tf_target = tf_eef * tf_translation;

      geometry_msgs::msg::Pose pose_i;
      tf2::toMsg(tf_target, pose_i);
      // 也可强制锁姿态为初始姿态（两者等价）：
      // pose_i.orientation = current_ps.pose.orientation;

      waypoints.push_back(pose_i);
    }

    // 打印“局部 Z 在世界的投影”，仅用于确认方向
    Eigen::Quaterniond q_we(current_ps.pose.orientation.w,
                            current_ps.pose.orientation.x,
                            current_ps.pose.orientation.y,
                            current_ps.pose.orientation.z);
    Eigen::Vector3d z_w = q_we.toRotationMatrix().col(2);
    RCLCPP_INFO(logger, "[InsertStrategy] Tool Z in world ~= [%.3f %.3f %.3f], steps=%zu, ref=%s, tip=%s",
                z_w.x(), z_w.y(), z_w.z(),
                waypoints.size(), current_ps.header.frame_id.c_str(), eef.c_str());
  }
  else if (mode == "world")
  {
    // 按世界 Z 离散（保持行为对称）
    const double total = insert_distance;
    const int steps = std::max(2, static_cast<int>(std::ceil(std::abs(total) / eef_step_param)) + 1);
    for (int i = 1; i <= steps; ++i) {
      double alpha = static_cast<double>(i) / steps;
      geometry_msgs::msg::Pose pose_i = current_ps.pose;
      pose_i.position.z += alpha * total;
      waypoints.push_back(pose_i);
    }
  }
  else
  {
    RCLCPP_WARN(logger, "[InsertStrategy] Unknown mode '%s', abort.", mode_in.c_str());
    return false; // 不再 fallback 到 world Z
  }

  // 规划（不再回退世界 Z）
  moveit_msgs::msg::RobotTrajectory traj_msg;
  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step_param, jump_threshold, traj_msg, avoid_collisions);

  RCLCPP_INFO(logger, "[InsertStrategy] Cartesian path fraction: %.1f%%", fraction * 100.0);
  if (fraction < 0.9) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Only %.1f%% of path planned (<90%%). No fallback. Abort.",
                 fraction * 100.0);
    return false;
  }

  // 可选：验收“直线 & 姿态”
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Failed to get current state for verification.");
    return false;
  }
  moveit::core::RobotModelConstPtr model = current_state->getRobotModel();

  robot_trajectory::RobotTrajectory rt(model, move_group.getName());
  rt.setRobotTrajectoryMsg(*current_state, traj_msg);

  const Eigen::Vector3d p0(current_ps.pose.position.x,
                           current_ps.pose.position.y,
                           current_ps.pose.position.z);
  const Eigen::Vector3d p1(waypoints.back().position.x,
                           waypoints.back().position.y,
                           waypoints.back().position.z);
  const Eigen::Quaterniond q0(current_ps.pose.orientation.w,
                              current_ps.pose.orientation.x,
                              current_ps.pose.orientation.y,
                              current_ps.pose.orientation.z);

  const bool keep_orientation = (mode == "joint");
  const double pos_tol = 0.002;          // 2 mm
  const double ang_tol = 5.0 * M_PI/180; // 5 度
  if (!verifyLinearInsertion(rt, eef, p0, p1, q0, keep_orientation, pos_tol, ang_tol)) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Verification failed. Abort.");
    return false;
  }

  // 时间参数化 & 执行
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, scale, scale)) {
    RCLCPP_ERROR(logger, "[InsertStrategy] Time parameterization failed.");
    return false;
  }
  rt.getRobotTrajectoryMsg(traj_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj_msg;

  auto exec_res = move_group.execute(plan);
  if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "[InsertStrategy] Insertion executed successfully!");
    return true;
  }
  RCLCPP_ERROR(logger, "[InsertStrategy] Motion execution failed!");
  return false;
}

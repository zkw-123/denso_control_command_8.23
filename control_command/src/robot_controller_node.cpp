#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>   // 保持你的头
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib> // std::strtod

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include "insert_strategy.hpp"

class RobotController : public rclcpp::Node {
public:
  RobotController()
  : Node("robot_controller",
         rclcpp::NodeOptions().use_intra_process_comms(false)
                              .parameter_overrides({{"use_sim_time", true}})),
    command_count_(0) {

    RCLCPP_INFO(this->get_logger(), "Using simulation time for Gazebo compatibility");

    RCLCPP_INFO(this->get_logger(), "Waiting for move_group to be ready...");
    waitForMoveGroup();

    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          std::shared_ptr<rclcpp::Node>(this), "arm");
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully!");
      printRuntimeFrames();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
      throw;
    }

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot_command", 10,
        std::bind(&RobotController::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Robot Controller Node Initialized");
    RCLCPP_INFO(this->get_logger(), "Listening on topic: /robot_command");
    RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to receive commands...");
    RCLCPP_INFO(this->get_logger(), "========================================");

    printSupportedCommands();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  uint32_t command_count_;

  // === 工具：向现有 path_constraints 里追加 Joint6 的 ±5° 走廊（以当前角度为中心）===
  bool appendJ6Corridor(moveit_msgs::msg::Constraints& path_constraints, double tol_j6_rad)
  {
    const std::vector<double> joints = move_group_->getCurrentJointValues();
    if (joints.size() < 6) {
      RCLCPP_ERROR(this->get_logger(), "Cannot add J6 corridor: joint vector size %zu (<6).", joints.size());
      return false;
    }
    moveit_msgs::msg::JointConstraint j6c;
    j6c.joint_name = "joint_6"; // TODO: 若真实名称为 "J6"，请改这里
    j6c.position = joints[5];   // 以“当前 J6”为中心
    j6c.tolerance_above = tol_j6_rad;
    j6c.tolerance_below = tol_j6_rad;
    j6c.weight = 1.0;
    path_constraints.joint_constraints.push_back(j6c);
    return true;
  }

  // === 工具：生成“Z向下 + J4走廊”的约束（yaw 放宽到 π，避免卡住）===
  moveit_msgs::msg::Constraints makeZDownConstraintWithJ4(double tol_j4_rad) {
    moveit_msgs::msg::Constraints cons;

    // J4 走廊（以当前角度为中心）
    const auto joints = move_group_->getCurrentJointValues();
    if (joints.size() >= 4) {
      moveit_msgs::msg::JointConstraint j4;
      j4.joint_name = "joint_4";           // TODO: 若真实名为 "J4" 请改这里
      j4.position = joints[3];
      j4.tolerance_above = tol_j4_rad;
      j4.tolerance_below = tol_j4_rad;
      j4.weight = 1.0;
      cons.joint_constraints.push_back(j4);
    } else {
      RCLCPP_WARN(this->get_logger(), "[horizental] joint vector size < 4, skip J4 corridor.");
    }

    // 目标姿态：Z 对齐世界 -Z（roll=0, pitch=pi, yaw 任意）
    Eigen::AngleAxisd Rz(0.0,            Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(M_PI,           Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(0.0,            Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_des = Rz * Ry * Rx;
    q_des.normalize();

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_des.x(); q_msg.y = q_des.y(); q_msg.z = q_des.z(); q_msg.w = q_des.w();

    // OrientationConstraint：roll/pitch 严一些，yaw 放到 π（完全自由）
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = "J6";                 // 末端 link 名
    oc.header.frame_id = "world";
    oc.orientation = q_msg;
    oc.absolute_x_axis_tolerance = 0.0349;  // ~2°
    oc.absolute_y_axis_tolerance = 0.0349;  // ~2°
    oc.absolute_z_axis_tolerance = M_PI;    // yaw 自由
    oc.weight = 1.0;

    cons.orientation_constraints.push_back(oc);
    return cons;
  }

  // ===== 通用设置 =====
  void applyCommonSetup() {
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    move_group_->setEndEffectorLink("J6");        // 若你的tip不同，请改
    move_group_->setPoseReferenceFrame("world");  // 统一世界系
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
  }

  void printRuntimeFrames() {
    RCLCPP_INFO(this->get_logger(), "group=%s", move_group_->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "planning_frame=%s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "eef_link=%s", move_group_->getEndEffectorLink().c_str());
  }

  void waitForMoveGroup() {
    while (rclcpp::ok()) {
      auto node_names = this->get_node_names();
      bool move_group_found = false;
      for (const auto &name : node_names) {
        if (name == "/move_group") { move_group_found = true; break; }
      }
      if (move_group_found) {
        RCLCPP_INFO(this->get_logger(), "move_group node found, checking parameters...");
        auto parameters_client =
            std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
        if (parameters_client->wait_for_service(std::chrono::seconds(5))) {
          try {
            auto parameters = parameters_client->get_parameters({"robot_description_semantic"});
            if (!parameters.empty() && !parameters[0].as_string().empty()) {
              RCLCPP_INFO(this->get_logger(), "robot_description_semantic parameter found!");
              RCLCPP_INFO(this->get_logger(), "Copying parameters to local node...");
              copyParametersFromMoveGroup();
              rclcpp::sleep_for(std::chrono::seconds(2));
              break;
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Error checking parameters: %s", e.what());
          }
        }
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for move_group and parameters...");
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }

  void copyParametersFromMoveGroup() {
    auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    try {
      auto params = parameters_client->get_parameters(
          {"robot_description", "robot_description_semantic"});
      if (params.size() >= 2) {
        this->declare_parameter("robot_description", params[0].as_string());
        this->declare_parameter("robot_description_semantic", params[1].as_string());
        RCLCPP_INFO(this->get_logger(), "Parameters copied successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get all required parameters");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error copying parameters: %s", e.what());
    }
  }

  void printSupportedCommands() {
    RCLCPP_INFO(this->get_logger(), "Supported Command Formats:");
    RCLCPP_INFO(this->get_logger(),
                "  pose <x> <y> <z>                  - Move to cartesian position (J4 corridor only + J6 ±5°)");
    RCLCPP_INFO(this->get_logger(),
                "  move <x> <y> <z>                  - Move to cartesian position (free orientation + J6 ±5°)");
    RCLCPP_INFO(this->get_logger(),
                "  joints <j1> <j2> ... <jN>         - Move to joint positions (J6 ±5° corridor)");
    RCLCPP_INFO(this->get_logger(),
                "  insert <distance> [percent] [joint|world] - Linear insertion");
    RCLCPP_INFO(this->get_logger(),
                "  horizental                      - Re-orient in place: tip faces world -Z (J4±45°, yaw free)");
    RCLCPP_INFO(this->get_logger(),
                "  horizental orient|here          - Same as above");
    RCLCPP_INFO(this->get_logger(),
                "  horizental <x> <y> <z>          - Move to (x,y,z) with tip facing world -Z (J4±45°, yaw free)");
    RCLCPP_INFO(this->get_logger(),
                "  horizental move <x> <y> <z>     - Same as above");
    RCLCPP_INFO(this->get_logger(),
                "  current                         - Get current robot status");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  // ====== pose/move/joints/insert 基础实现 ======

  void moveToPosition(double x, double y, double z) {
    applyCommonSetup();
    RCLCPP_INFO(this->get_logger(), "Setting target position (free orientation, with J6 corridor ±5°)...");
    move_group_->getCurrentState(10.0);

    // 添加 J6 走廊（±5°）
    moveit_msgs::msg::Constraints path_constraints;
    appendJ6Corridor(path_constraints, 5.0 * M_PI / 180.0);
    move_group_->setPathConstraints(path_constraints);

    move_group_->setPlanningTime(20.0);
    move_group_->setNumPlanningAttempts(15);
    move_group_->setGoalTolerance(0.01); // ~1cm
    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success) {
      auto exec_res = move_group_->execute(plan);
      if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Position-only planning failed!");
    }

    move_group_->clearPathConstraints();   // 清理
    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  bool planPositionWithJ4Corridor(double x, double y, double z, double tol_j4_rad) {
    move_group_->setStartStateToCurrentState();

    const std::vector<double> current_joints = move_group_->getCurrentJointValues();
    if (current_joints.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Current joint vector size %zu (<4).", current_joints.size());
      return false;
    }

    moveit_msgs::msg::Constraints path_constraints;

    // J4 走廊
    moveit_msgs::msg::JointConstraint jcm;
    jcm.joint_name = "joint_4";      // TODO: 若真实名称为 "J4" 请改这里
    jcm.position = current_joints[3];
    jcm.tolerance_above = tol_j4_rad;
    jcm.tolerance_below = tol_j4_rad;
    jcm.weight = 1.0;
    path_constraints.joint_constraints.push_back(jcm);

    // J6 走廊（±5°）
    appendJ6Corridor(path_constraints, 5.0 * M_PI / 180.0);

    move_group_->setPathConstraints(path_constraints);

    move_group_->setPlanningTime(10.0);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01745);
    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok) {
      auto exec = move_group_->execute(plan);
      ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed under J4 corridor.");
    }

    move_group_->clearPathConstraints();   // 清理
    move_group_->clearPoseTargets();
    return ok;
  }

  void moveToPose(double x, double y, double z) {
    applyCommonSetup();
    RCLCPP_INFO(this->get_logger(), "Setting target position with J4 corridor only (+ J6 ±5°)...");
    move_group_->getCurrentState(10.0);

    const std::vector<double> tol_j4_list = {0.17, 0.35, 0.52, 0.87};  // 10°,20°,30°,50°
    bool done = false;

    for (double tol_j4 : tol_j4_list) {
      RCLCPP_INFO(this->get_logger(), "Trying J4 corridor: %.1f deg", tol_j4 * 180.0 / M_PI);
      if (planPositionWithJ4Corridor(x, y, z, tol_j4)) { done = true; break; }
      RCLCPP_WARN(this->get_logger(), "Plan failed with corridor=%.1f deg, relaxing...",
                  tol_j4 * 180.0 / M_PI);
    }

    if (!done) {
      RCLCPP_WARN(this->get_logger(), "Fallback: no J4 constraint (but keep J6 ±5°).");
      // 仅加 J6 走廊
      moveit_msgs::msg::Constraints path_constraints;
      appendJ6Corridor(path_constraints, 5.0 * M_PI / 180.0);
      move_group_->setPathConstraints(path_constraints);

      move_group_->setPositionTarget(x, y, z);
      move_group_->setGoalPositionTolerance(0.01);
      move_group_->setGoalOrientationTolerance(0.01745);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        auto exec = move_group_->execute(plan);
        if (exec != moveit::core::MoveItErrorCode::SUCCESS)
          RCLCPP_ERROR(this->get_logger(), "Execution failed in fallback!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed in fallback!");
      }

      move_group_->clearPathConstraints(); // 清理
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void moveToJoints(const std::vector<double> &joints) {
    applyCommonSetup();
    RCLCPP_INFO(this->get_logger(), "Setting target joint positions (with J6 corridor ±5°)...");
    // 添加 J6 走廊（±5°）
    moveit_msgs::msg::Constraints path_constraints;
    appendJ6Corridor(path_constraints, 5.0 * M_PI / 180.0);
    move_group_->setPathConstraints(path_constraints);

    move_group_->setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success) {
      (void)move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Joint planning failed!");
    }

    move_group_->clearPathConstraints(); // 清理
  }

  void insertAlongWorldZ(double dz, double percent) {
    applyCommonSetup();
    RCLCPP_INFO(this->get_logger(),
                "Fallback: Cartesian path along WORLD Z, dz=%.4f m (%.1f%%) with J6 corridor ±5°",
                dz, percent);

    // 添加 J6 走廊（±5°）——注意：computeCartesianPath 通常不会应用 path constraints
    moveit_msgs::msg::Constraints path_constraints;
    appendJ6Corridor(path_constraints, 5.0 * M_PI / 180.0);
    move_group_->setPathConstraints(path_constraints);

    double scale = std::min(1.0, std::max(0.01, percent / 100.0));
    move_group_->setMaxVelocityScalingFactor(scale);
    move_group_->setMaxAccelerationScalingFactor(scale);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose("J6").pose;

    geometry_msgs::msg::Pose target = current_pose;
    target.position.z += dz; // 世界Z方向
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;

    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "Path planning result: %.1f%% of path achieved",
                fraction * 100.0);

    if (fraction > 0.9) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      auto exec_res = move_group_->execute(plan);
      if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Execution failed for world-Z insertion!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Cartesian path planning failed (world Z). Only %.1f%% planned.",
                   fraction * 100.0);
    }

    move_group_->clearPathConstraints(); // 清理
  }

  void getCurrentStatus() {
    RCLCPP_INFO(this->get_logger(), "=== ROBOT CURRENT STATUS ===");
    try {
      move_group_->getCurrentState(10.0);
      auto current_pose = move_group_->getCurrentPose("J6").pose;
      RCLCPP_INFO(this->get_logger(), "Pos: (%.4f, %.4f, %.4f)",
                  current_pose.position.x, current_pose.position.y, current_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Quat: (%.4f, %.4f, %.4f, %.4f)",
                  current_pose.orientation.x, current_pose.orientation.y,
                  current_pose.orientation.z, current_pose.orientation.w);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current status: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "========================");
  }

  // ====== 新增：horizental（使用 Z-down 约束 + J4 走廊，且不再限制 J6±5°） ======

  bool horizentalOrientHere() {
    applyCommonSetup();
    auto cur = move_group_->getCurrentPose("J6").pose;

    const double tol_j4 = 45.0 * M_PI / 180.0;
    moveit_msgs::msg::Constraints cons = makeZDownConstraintWithJ4(tol_j4);
    move_group_->setPathConstraints(cons);

    // 固定位置在当前点，姿态靠 OrientationConstraint 限制
    move_group_->setPositionTarget(cur.position.x, cur.position.y, cur.position.z);
    move_group_->setGoalPositionTolerance(0.002);       // ~2mm
    move_group_->setGoalOrientationTolerance(0.01745);  // ~1°（帮助 IK 收敛）

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "[horizental] orient-here planning failed.");
      move_group_->clearPathConstraints();
      move_group_->clearPoseTargets();
      return false;
    }

    auto exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    return exec_ok;
  }

  bool horizentalMoveTo(double x, double y, double z) {
    applyCommonSetup();

    const double tol_j4 = 45.0 * M_PI / 180.0;
    moveit_msgs::msg::Constraints cons = makeZDownConstraintWithJ4(tol_j4);
    move_group_->setPathConstraints(cons);

    move_group_->setPositionTarget(x, y, z);
    move_group_->setGoalPositionTolerance(0.002);       // ~2mm
    move_group_->setGoalOrientationTolerance(0.01745);  // ~1°

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "[horizental] move planning failed.");
      move_group_->clearPathConstraints();
      move_group_->clearPoseTargets();
      return false;
    }

    auto exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    return exec_ok;
  }

  // ====== 命令解析 ======
  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    command_count_++;
    std::string command = msg->data;

    RCLCPP_INFO(this->get_logger(), "[Command #%d] Input received: '%s'",
                command_count_, command.c_str());

    std::istringstream iss(command);
    std::string type; iss >> type;
    if (type.empty()) { RCLCPP_WARN(this->get_logger(), "Empty command"); return; }

    RCLCPP_INFO(this->get_logger(), "Command parsed: '%s'", type.c_str());
    RCLCPP_INFO(this->get_logger(), "Starting control execution...");

    auto start_time = this->get_clock()->now();

    if (type == "pose") {
      double x,y,z; if (iss >> x >> y >> z) moveToPose(x,y,z);
      else RCLCPP_ERROR(this->get_logger(), "Expected: pose <x> <y> <z>");
    }
    else if (type == "move") {
      double x,y,z; if (iss >> x >> y >> z) moveToPosition(x,y,z);
      else RCLCPP_ERROR(this->get_logger(), "Expected: move <x> <y> <z>");
    }
    else if (type == "joints") {
      std::vector<double> joints; double v; while (iss >> v) joints.push_back(v);
      if (!joints.empty()) moveToJoints(joints);
      else RCLCPP_ERROR(this->get_logger(), "Expected: joints <j1> <j2> ...");
    }
    else if (type == "insert") {
      double dz; double percent = 20.0; std::string mode = "joint";
      if (iss >> dz) { (void)(iss >> percent); (void)(iss >> mode);
        if (!(percent > 0.0 && percent <= 100.0)) {
          RCLCPP_ERROR(this->get_logger(),"Invalid percent %.1f", percent);
        } else {
          move_group_->setMaxVelocityScalingFactor(std::min(1.0, std::max(0.01, percent/100.0)));
          move_group_->setMaxAccelerationScalingFactor(std::min(1.0, std::max(0.01, percent/100.0)));
          bool ok = InsertStrategy::execute(*move_group_, this->get_logger(), dz, percent, mode);
          if (!ok) { RCLCPP_ERROR(this->get_logger(), "Insert failed."); }
        }
      } else RCLCPP_ERROR(this->get_logger(),
                          "Expected: insert <distance> [percent] [joint|world]");
    }
    else if (type == "horizental") {
      std::string token; double x,y,z;
      if (!(iss >> token)) {
        if (!horizentalOrientHere()) RCLCPP_ERROR(this->get_logger(), "horizental failed.");
      } else if (token == "orient" || token == "here") {
        if (!horizentalOrientHere()) RCLCPP_ERROR(this->get_logger(), "horizental orient failed.");
      } else if (token == "move") {
        if (iss >> x >> y >> z) {
          if (!horizentalMoveTo(x,y,z)) RCLCPP_ERROR(this->get_logger(), "horizental move failed.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Expected: horizental move <x> <y> <z>");
        }
      } else {
        char* endptr=nullptr; double maybe_x = std::strtod(token.c_str(), &endptr);
        if (endptr && *endptr=='\0') {
          x = maybe_x;
          if (iss >> y >> z) {
            if (!horizentalMoveTo(x,y,z)) RCLCPP_ERROR(this->get_logger(), "horizental <x y z> failed.");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Expected: horizental <x> <y> <z>");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Unknown subcommand for 'horizental': %s", token.c_str());
        }
      }
    }
    else if (type == "current" || type == "status") {
      getCurrentStatus();
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown command: '%s'", type.c_str());
      RCLCPP_INFO(this->get_logger(), "Use one of: pose, move, joints, insert, horizental, current");
    }

    auto end_time = this->get_clock()->now();
    auto duration = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Command #%d execution time: %.2f s",
                command_count_, duration.seconds());
    RCLCPP_INFO(this->get_logger(), "========================================");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

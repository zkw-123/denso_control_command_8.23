#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "insert_strategy.hpp"

// 只需要关节约束相关的头
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

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

      // 打印运行时的规划组/参考系/末端，确认一致性
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

  // 仅 J4 关节走廊：限制“路径上 J4 偏离当前角度不超过 tol_j4_rad”
  // 注意：如果你的第4关节名字不是 "joint_4"（比如 "J4"），请把下面的 joint 名改成你实际的名字。
  bool planPositionWithJ4Corridor(double x, double y, double z,
                                  double tol_j4_rad)
  {
    // 每次尝试都从“当前状态”起步
    move_group_->setStartStateToCurrentState();

    // 读取当前关节角（用于以当前 J4 为中心建“走廊”）
    const std::vector<double> current_joints = move_group_->getCurrentJointValues();
    if (current_joints.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Current joint vector has size %zu (<4).", current_joints.size());
      return false;
    }

    moveit_msgs::msg::Constraints path_constraints;

    moveit_msgs::msg::JointConstraint jcm;
    jcm.joint_name = "joint_4";              // TODO: 如你的机器人是 "J4" 则改成 "J4"
    jcm.position = current_joints[3];        // 以当前 J4 为中心
    jcm.tolerance_above = tol_j4_rad;        // 上下容差相同
    jcm.tolerance_below = tol_j4_rad;
    jcm.weight = 1.0;
    path_constraints.joint_constraints.push_back(jcm);

    move_group_->setPathConstraints(path_constraints);

    // 有路径约束会更慢，给足时间
    move_group_->setPlanningTime(10.0);

    // 仍然只给位置目标（姿态完全自由，J6 不再有任何约束）
    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (ok) {
      auto exec = move_group_->execute(plan);
      ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // 清掉约束，避免影响后续规划
    move_group_->clearPathConstraints();
    return ok;
  }

  // ===== 统一的规划前设置 =====
  void applyCommonSetup() {
    // 清理上次的目标/约束，避免交叉影响
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();

    // 起点/末端/参考系：固定三件事
    move_group_->setStartStateToCurrentState();   // 起点=当前状态
    move_group_->setEndEffectorLink("J6");        // 你的末端执行器就是 J6
    move_group_->setPoseReferenceFrame("world");  // 统一使用 world（如你习惯 base_link 可改之，但务必全程一致）

    // 让行为更稳定接近 RViz
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(10);

    // 温和的速度/加速度缩放（按需改）
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
  }

  void printRuntimeFrames() {
    RCLCPP_INFO(this->get_logger(), "group=%s", move_group_->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "planning_frame=%s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "eef_link=%s", move_group_->getEndEffectorLink().c_str());
  }

  void waitForMoveGroup() {
    // Wait for move_group node & params
    while (rclcpp::ok()) {
      auto node_names = this->get_node_names();
      bool move_group_found = false;
      for (const auto &name : node_names) {
        if (name == "/move_group") {
          move_group_found = true;
          break;
        }
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
                "  pose <x> <y> <z>                  - Move to cartesian position (J4 corridor only)");
    RCLCPP_INFO(this->get_logger(),
                "  move <x> <y> <z>                  - Move to cartesian position (free orientation)");
    RCLCPP_INFO(this->get_logger(),
                "  joints <j1> <j2> ... <jN>         - Move to joint positions");
    RCLCPP_INFO(this->get_logger(),
                "  insert <distance> [percent] [joint|world] - Linear insertion");
    RCLCPP_INFO(this->get_logger(),
                "       distance: meters (+ along +Z, - along -Z)");
    RCLCPP_INFO(this->get_logger(),
                "       percent : 0~100 max velocity scaling (default 20)");
    RCLCPP_INFO(this->get_logger(),
                "       mode    : joint (J6 local Z, default) | world (world Z)");
    RCLCPP_INFO(this->get_logger(),
                "  current                           - Get current robot status");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    command_count_++;
    std::string command = msg->data;

    RCLCPP_INFO(this->get_logger(), "[Command #%d] Input received: '%s'", command_count_,
                command.c_str());

    std::istringstream iss(command);
    std::string type;
    iss >> type;

    if (type.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty command received");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Command parsed successfully, type: '%s'", type.c_str());
    RCLCPP_INFO(this->get_logger(), "Starting control execution...");

    auto start_time = this->get_clock()->now();

    if (type == "pose") {
      double x, y, z;
      if (iss >> x >> y >> z) {
        moveToPose(x, y, z);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose parameters. Expected: pose <x> <y> <z>");
      }
    } else if (type == "joints") {
      std::vector<double> joints;
      double val;
      while (iss >> val) {
        joints.push_back(val);
      }
      if (!joints.empty()) {
        moveToJoints(joints);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "No joint values provided. Expected: joints <j1> <j2> ... <jN>");
      }
    } else if (type == "insert") {
      double dz;
      double percent = 20.0; // default 20%
      std::string mode = "joint";
      if (iss >> dz) {
        if (iss >> percent) {
          // optional
        }
        if (iss >> mode) {
          // optional
        }
        // sanitize
        if (!(percent > 0.0 && percent <= 100.0)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid percent %.1f (valid range 0~100].", percent);
          return;
        }
        if (mode != "joint" && mode != "world") {
          RCLCPP_WARN(this->get_logger(),
                      "Unknown mode '%s', fallback to 'joint'.", mode.c_str());
          mode = "joint";
        }

        RCLCPP_INFO(this->get_logger(),
                    "Insert request: distance=%.4f m, percent=%.1f%%, mode=%s",
                    dz, percent, mode.c_str());

        move_group_->setMaxVelocityScalingFactor(std::min(1.0, std::max(0.01, percent / 100.0)));
        move_group_->setMaxAccelerationScalingFactor(
            std::min(1.0, std::max(0.01, percent / 100.0)));

        bool ok = InsertStrategy::execute(*move_group_, this->get_logger(), dz, percent, mode);
        if (!ok) {
          if (mode == "joint") {
            RCLCPP_WARN(this->get_logger(),
                        "Insert (joint Z) failed, trying fallback along world Z...");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Insert (world Z) failed, trying simple cartesian fallback...");
          }
          insertAlongWorldZ(dz, percent);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid insert parameter. Expected: insert <distance> [percent] [joint|world]");
      }
    } else if (type == "move") {
      double x, y, z;
      if (iss >> x >> y >> z) {
        moveToPosition(x, y, z);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid move parameters. Expected: move <x> <y> <z>");
      }
    } else if (type == "current" || type == "status") {
      getCurrentStatus();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown command type: '%s'", type.c_str());
      RCLCPP_INFO(this->get_logger(), "Use one of: pose, move, joints, insert, current");
    }

    auto end_time = this->get_clock()->now();
    auto duration = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Command #%d execution time: %.2f seconds", command_count_,
                duration.seconds());
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  // ===== pose：只给位置 + 仅 J4 路径约束（逐步放宽）=====
  void moveToPose(double x, double y, double z) {
    applyCommonSetup();  // 统一设置（起点/EEF/参考系/规划器等）

    RCLCPP_INFO(this->get_logger(), "Setting target position with J4 corridor only...");
    move_group_->getCurrentState(10.0);

    // 递增放宽 J4 走廊（单位：弧度）
    const std::vector<double> tol_j4_list = {0.17, 0.35, 0.52, 0.87};  // 10°, 20°, 30°, 50°
    bool done = false;

    for (double tol_j4 : tol_j4_list) {
      RCLCPP_INFO(this->get_logger(), "Trying J4 corridor: %.1f deg", tol_j4 * 180.0 / M_PI);
      if (planPositionWithJ4Corridor(x, y, z, tol_j4)) {
        done = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Plan failed with J4 corridor=%.1f deg, relaxing...",
                  tol_j4 * 180.0 / M_PI);
    }

    if (!done) {
      // 兜底：完全不加约束（提示：此时 J4 可能会动得多）
      RCLCPP_WARN(this->get_logger(), "Fallback: no J4 constraint (may rotate J4 freely).");
      move_group_->setPositionTarget(x, y, z);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        auto exec = move_group_->execute(plan);
        if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
          done = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Execution failed in fallback!");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed in fallback!");
      }
    }

    if (done) {
      auto final_pose = move_group_->getCurrentPose("J6").pose;
      RCLCPP_INFO(this->get_logger(), "Final position: (%.3f, %.3f, %.3f)",
                  final_pose.position.x, final_pose.position.y, final_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Final orientation: (%.3f, %.3f, %.3f, %.3f)",
                  final_pose.orientation.x, final_pose.orientation.y,
                  final_pose.orientation.z, final_pose.orientation.w);
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  // move：保留“完全自由”的位置目标
  void moveToPosition(double x, double y, double z) {
    applyCommonSetup();  // 统一设置

    RCLCPP_INFO(this->get_logger(), "Setting target position (free orientation)...");
    move_group_->getCurrentState(10.0);

    move_group_->setPlanningTime(20.0);
    move_group_->setNumPlanningAttempts(15);
    move_group_->setGoalTolerance(0.01);

    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success) {
      auto exec_res = move_group_->execute(plan);
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Position motion completed successfully!");
        auto final_pose = move_group_->getCurrentPose("J6").pose;
        RCLCPP_INFO(this->get_logger(), "Final position: (%.3f, %.3f, %.3f)",
                    final_pose.position.x, final_pose.position.y, final_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Final orientation: (%.3f, %.3f, %.3f, %.3f)",
                    final_pose.orientation.x, final_pose.orientation.y,
                    final_pose.orientation.z, final_pose.orientation.w);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Position-only planning failed!");
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void moveToJoints(const std::vector<double> &joints) {
    applyCommonSetup();  // 统一设置

    RCLCPP_INFO(this->get_logger(), "Setting target joint positions...");
    auto current_joints = move_group_->getCurrentJointValues();
    for (size_t i = 0; i < current_joints.size() && i < joints.size(); ++i) {
      double diff = joints[i] - current_joints[i];
      RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f -> %.3f (delta %.3f rad)", i + 1,
                  current_joints[i], joints[i], diff);
    }

    move_group_->setJointValueTarget(joints);
    bool success = static_cast<bool>(move_group_->move());
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Joint motion completed successfully!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Joint motion failed!");
    }
  }

  // Fallback: world/base frame Z 方向直线插补（保持之前实现）
  void insertAlongWorldZ(double dz, double percent) {
    applyCommonSetup();  // 统一设置

    RCLCPP_INFO(this->get_logger(),
                "Fallback: Cartesian path along WORLD Z, dz=%.4f m (%.1f%%)", dz, percent);

    // 速度/加速度缩放
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
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Linear insertion (world Z) executed successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed for world-Z insertion!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Cartesian path planning failed (world Z). Only %.1f%% planned.",
                   fraction * 100.0);
    }
  }

  void getCurrentStatus() {
    RCLCPP_INFO(this->get_logger(), "=== ROBOT CURRENT STATUS ===");

    try {
      move_group_->getCurrentState(10.0);

      auto current_pose = move_group_->getCurrentPose("J6").pose;
      RCLCPP_INFO(this->get_logger(), "Current End-Effector Position:");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f m", current_pose.position.x);
      RCLCPP_INFO(this->get_logger(), "  Y: %.4f m", current_pose.position.y);
      RCLCPP_INFO(this->get_logger(), "  Z: %.4f m", current_pose.position.z);

      RCLCPP_INFO(this->get_logger(), "Current Orientation (Quaternion):");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f", current_pose.orientation.x);
      RCLCPP_INFO(this->get_logger(), "  Y: %.4f", current_pose.orientation.y);
      RCLCPP_INFO(this->get_logger(), "  Z: %.4f", current_pose.orientation.z);
      RCLCPP_INFO(this->get_logger(), "  W: %.4f", current_pose.orientation.w);

      auto current_joints = move_group_->getCurrentJointValues();
      RCLCPP_INFO(this->get_logger(), "Current Joint Positions (rad):");
      for (size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.4f", i + 1, current_joints[i]);
      }

      double x = current_pose.position.x;
      double y = current_pose.position.y;
      double z = current_pose.position.z;

      RCLCPP_INFO(this->get_logger(), "=== SUGGESTED NEARBY TARGETS ===");
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move +5cm in X)", x + 0.05, y, z);
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move -5cm in X)", x - 0.05, y, z);
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move +5cm in Y)", x, y + 0.05, z);
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move -5cm in Y)", x, y - 0.05, z);
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move +5cm in Z)", x, y, z + 0.05);
      RCLCPP_INFO(this->get_logger(), "  pose %.3f %.3f %.3f  (move -5cm in Z)", x, y, z - 0.05);

      RCLCPP_INFO(this->get_logger(), "Insert examples:");
      RCLCPP_INFO(this->get_logger(), "  insert 0.02           (20%%, joint Z default)");
      RCLCPP_INFO(this->get_logger(), "  insert 0.02 30 joint  (30%%, joint Z)");
      RCLCPP_INFO(this->get_logger(), "  insert 0.02 20 world  (20%%, world Z)");

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current status: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "========================");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Robot Controller Node...");

  try {
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Robot Controller Node shutting down...");
  rclcpp::shutdown();
  return 0;
}

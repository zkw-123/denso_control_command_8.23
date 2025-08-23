#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logger.hpp>
#include <string>

/**
 * @brief 插入策略类，提供沿Z轴进行线性插入的功能
 */
class InsertStrategy
{
public:
  /**
   * @brief 执行插入操作
   * @param move_group MoveIt移动组接口
   * @param logger ROS日志记录器
   * @param insert_distance 插入距离（沿Z轴，单位：米）
   * @param percent 速度缩放百分比 (0~100)
   * @param mode 插入模式： "joint" = 末端执行器(J6)局部Z方向, "world" = 世界坐标系Z方向
   * @return true 插入成功，false 插入失败
   */
  static bool execute(moveit::planning_interface::MoveGroupInterface &move_group,
                      rclcpp::Logger logger,
                      double insert_distance,
                      double percent,
                      const std::string &mode);
};

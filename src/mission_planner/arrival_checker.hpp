#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include "arrival_checker_core.hpp"

namespace autoware::mission_planner_universe
{

using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;

class ArrivalChecker
{
public:
  explicit ArrivalChecker(rclcpp::Node * node);

  /// route clear 등으로 goal 을 지우는 경우
  void set_goal();  // 기존 시그니처 유지

  /// goal 설정 (uuid 포함 메시지)
  void set_goal(const PoseWithUuidStamped & goal);

  /// 현재 pose 기준 도착 여부 판단
  bool is_arrived(const PoseStamped & pose) const;

private:
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;
  ArrivalCheckerCore core_;
};

}  // namespace autoware::mission_planner_universe

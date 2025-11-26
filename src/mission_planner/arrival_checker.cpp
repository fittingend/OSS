// Copyright 2022 TIER IV, Inc.
// (라이선스 헤더 동일)

#include "arrival_checker.hpp"

#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <tf2/utils.h>

namespace autoware::mission_planner_universe
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node)
: vehicle_stop_checker_(node),
  core_(
    // 각도: 파라미터(deg) → rad 변환
    autoware::universe_utils::deg2rad(
      node->declare_parameter<double>("arrival_check_angle_deg")),
    // 거리
    node->declare_parameter<double>("arrival_check_distance"),
    // 정지 확인 시간
    node->declare_parameter<double>("arrival_check_duration"),
    // StopCheckerFn: VehicleStopChecker 에 위임
    [this](double duration_sec) {
      return vehicle_stop_checker_.isVehicleStopped(duration_sec);
    })
{
}

void ArrivalChecker::set_goal()
{
  // Ignore the modified goal after the route is cleared.
  core_.clear_goal();
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  Pose2D core_goal;
  // PoseWithUuidStamped: header + pose(geometry_msgs::msg::Pose) + uuid
  core_goal.x = goal.pose.position.x;
  core_goal.y = goal.pose.position.y;
  core_goal.yaw = tf2::getYaw(goal.pose.orientation);
  core_goal.frame_id = goal.header.frame_id;

  core_.set_goal(core_goal);
}

bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  Pose2D core_pose;
  core_pose.x = pose.pose.position.x;
  core_pose.y = pose.pose.position.y;
  core_pose.yaw = tf2::getYaw(pose.pose.orientation);
  core_pose.frame_id = pose.header.frame_id;

  return core_.is_arrived(core_pose);
}

}  // namespace autoware::mission_planner_universe

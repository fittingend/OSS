#include "arrival_checker_core.hpp"

#include <autoware/universe_utils/math/normalization.hpp>

namespace autoware::mission_planner_universe
{

ArrivalCheckerCore::ArrivalCheckerCore(
  double angle_rad,
  double distance,
  double duration_sec,
  StopCheckerFn stop_checker)
: angle_(std::fabs(angle_rad)),
  distance_(std::fabs(distance)),
  duration_(std::fabs(duration_sec)),
  stop_checker_(std::move(stop_checker))
{
}

void ArrivalCheckerCore::clear_goal()
{
  goal_.reset();
}

void ArrivalCheckerCore::set_goal(const Pose2D & goal)
{
  goal_ = goal;
}

bool ArrivalCheckerCore::is_arrived(const Pose2D & pose) const
{
  if (!goal_) {
    return false;
  }

  const auto & goal = *goal_;

  // frame id 체크
  if (goal.frame_id != pose.frame_id) {
    return false;
  }

  // 거리 체크
  const double dx = pose.x - goal.x;
  const double dy = pose.y - goal.y;
  const double dist = std::hypot(dx, dy);
  if (dist > distance_) {
    return false;
  }

  // 각도 체크
  const double yaw_diff =
    autoware::universe_utils::normalizeRadian(pose.yaw - goal.yaw);
  if (std::fabs(yaw_diff) > angle_) {
    return false;
  }

  // 차량 정지 여부 체크
  if (stop_checker_) {
    return stop_checker_(duration_);
  }

  // stop_checker 가 없으면 정지 조건은 패스 (true)라고 가정
  return true;
}

}  // namespace autoware::mission_planner_universe

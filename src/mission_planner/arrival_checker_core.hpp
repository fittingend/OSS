#pragma once

#include <cmath>
#include <functional>
#include <optional>
#include <string>

namespace autoware::mission_planner_universe
{

/// ROS/geometry_msgs 없이 쓰기 위한 단순 Pose 구조체
struct Pose2D
{
  double x{};
  double y{};
  double yaw{};          // [rad]
  std::string frame_id;  // "map", "base_link" 등
};

/// VehicleStopChecker 역할을 추상화한 콜백을 받아서 동작하는 코어 로직
class ArrivalCheckerCore
{
public:
  /// duration_sec 동안 차량이 정지했는지 판단하는 콜백
  using StopCheckerFn = std::function<bool(double duration_sec)>;

  ArrivalCheckerCore(
    double angle_rad,
    double distance,
    double duration_sec,
    StopCheckerFn stop_checker);

  /// 목표 제거 (route clear 시 호출)
  void clear_goal();

  /// 목표 설정
  void set_goal(const Pose2D & goal);

  bool has_goal() const noexcept { return goal_.has_value(); }

  /// 현재 pose 기준 도착 여부 판단
  bool is_arrived(const Pose2D & pose) const;

private:
  double angle_;      // 허용 각도 차 [rad]
  double distance_;   // 허용 거리 [m]
  double duration_;   // 정지 판정용 시간 [sec]
  StopCheckerFn stop_checker_;
  std::optional<Pose2D> goal_;
};

}  // namespace autoware::mission_planner_universe

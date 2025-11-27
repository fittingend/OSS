#pragma once

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{

using LaneletRoute  = autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = decltype(LaneletRoute::segments);
using RoutePoints   = std::vector<geometry_msgs::msg::Pose>;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;

using Polygon2d    = autoware::universe_utils::Polygon2d;
using MultiPolygon2d = autoware::universe_utils::MultiPolygon2d;
using LinearRing2d = autoware::universe_utils::LinearRing2d;

/**
 * @brief lanelet2 default planner 에서 사용하는 파라미터 집합
 *
 * Autoware default_planner 의 param_ 구조를 core 레벨로 분리한 것.
 */
struct DefaultPlannerParam
{
  //! lanelet 진행 방향과 goal yaw 차이 허용치[deg]
  double goal_angle_threshold_deg{10.0};

  //! goal pose 를 centerline 기반으로 보정할지 여부
  bool enable_correct_goal_pose{true};

  //! no_drivable lanelet(갓길/보조차선 등)도 경로계획에 포함할지 여부
  bool consider_no_drivable_lanes{false};

  //! 차량 footprint 가 lanelet 영역 밖으로 나가면 invalid 처리할지 여부
  bool check_footprint_inside_lanes{true};
};

/**
 * @brief Lanelet2 기반 default 경로계획 알고리즘의 코어 모듈
 *
 * - ROS(rclcpp, publisher/subscriber, pluginlib) 에 전혀 의존하지 않음
 * - Autoware route_handler / vehicle_info_utils / universe_utils / lanelet2 만 의존
 *
 * Adaptive AUTOSAR MissionPlannerApp 에서 그대로 링크해서 사용 가능.
 */
class DefaultPlannerCore
{
public:
  DefaultPlannerCore(
    const DefaultPlannerParam & param,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

  /// lanelet2 맵 설정
  void setMap(const LaneletMapBin & msg);

  /// 그래프 준비(맵 로딩 완료) 여부
  bool ready() const noexcept { return is_graph_ready_; }

  /// 체크포인트(waypoints) 기반 경로계획
  LaneletRoute plan(const RoutePoints & points);

  /// 현재 route 를 RouteHandler 에 반영
  void updateRoute(const LaneletRoute & route);

  /// route 초기화
  void clearRoute();

private:
  DefaultPlannerParam param_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  autoware::route_handler::RouteHandler route_handler_;
  bool is_graph_ready_{false};

  /// goal pose 가 유효한 위치/자세인지 검사
  bool isGoalValid(
    const geometry_msgs::msg::Pose & goal,
    const lanelet::ConstLanelets & path_lanelets) const;

  /// 차량 footprint 가 lanelet/shoulder 영역 안에 들어오는지 검사
  bool checkGoalFootprintInsideLanes(
    const lanelet::ConstLanelet & closest_lanelet_to_goal,
    const lanelet::ConstLanelets & path_lanelets,
    const Polygon2d & goal_footprint) const;

  /// goal pose 의 z 값을 lanelet 높이에 맞춰 보정
  geometry_msgs::msg::Pose refineGoalHeight(
    const geometry_msgs::msg::Pose & goal,
    const RouteSections & route_sections) const;

  /// start_lanelet 기준으로 up_to_distance 만큼 이어지는 lanelet 들 수집
  static lanelet::ConstLanelets nextLaneletsUpTo(
    const lanelet::ConstLanelet & start_lanelet,
    double up_to_distance,
    const autoware::route_handler::RouteHandler & route_handler);
};

}  // namespace autoware::mission_planner_universe::lanelet2

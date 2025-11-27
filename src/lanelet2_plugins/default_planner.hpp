// default_planner.hpp
#pragma once

#include "default_planner_core.hpp"         // DefaultPlannerCore
#include "mission_planner_core.hpp"         // PlannerPlugin, RoutePoints, LaneletRoute (환경에 맞게 조정)

#include <autoware_auto_mapping_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware::mission_planner_universe::lanelet2
{

using autoware_auto_mapping_msgs::msg::LaneletMapBin;
using visualization_msgs::msg::MarkerArray;

/**
 * @brief Lanelet2 Default Planner 의 ROS / pluginlib 래퍼
 *
 * - 기존 Autoware `DefaultPlanner` 클래스와 같은 plugin 이름 유지
 * - ROS 파라미터, 토픽 subscribe/publish, marker 시각화 담당
 * - 실제 경로계획 알고리즘은 DefaultPlannerCore 에 위임
 */
class DefaultPlanner : public PlannerPlugin
{
public:
  using RoutePoints  = PlannerPlugin::RoutePoints;
  using LaneletRoute = PlannerPlugin::LaneletRoute;

  DefaultPlanner() = default;

  /// pluginlib 에서 호출하는 기본 초기화
  void initialize(rclcpp::Node * node) override;

  /// 단위 테스트/오프라인용: 초기 맵 메시지를 바로 넣는 초기화
  void initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg);

  /// 맵 로딩/그래프 준비 여부
  bool ready() const override;

  /// 체크포인트 기반 경로계획
  LaneletRoute plan(const RoutePoints & points) override;

  /// 외부에서 route 를 업데이트 할 때 (예: reroute)
  void updateRoute(const LaneletRoute & route) override;

  /// route 클리어
  void clearRoute() override;

  /// 시각화용 MarkerArray (간단 버전)
  MarkerArray visualize(const RoutePoints & points) override;

private:
  // ROS 노드 포인터 (소유권 없음)
  rclcpp::Node * node_{nullptr};

  // Core 알고리즘
  std::shared_ptr<DefaultPlannerCore> core_;

  // 맵 subscriber
  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;

  // goal footprint 디버그용 publisher
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;

  // 공통 초기화 (파라미터/vehicle_info 등)
  void initializeCommon(rclcpp::Node * node);

  // 맵 콜백
  void onMap(const LaneletMapBin::ConstSharedPtr msg);

  // 마지막 goal pose 기준 footprint 디버그 Marker 생성
  MarkerArray makeGoalFootprintMarkers(const RoutePoints & points);
};

}  // namespace autoware::mission_planner_universe::lanelet2

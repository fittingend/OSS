// default_planner.cpp

#include "default_planner.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware::mission_planner_universe::lanelet2
{

using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::Polygon2d;
using autoware::universe_utils::pose2transform;
using autoware::universe_utils::transformVector;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createPoint;

void DefaultPlanner::initializeCommon(rclcpp::Node * node)
{
  node_ = node;

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_goal_footprint_marker_ =
    node_->create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

  // vehicle_info & param 로딩 후 Core 생성
  auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();

  DefaultPlannerParam param;
  param.goal_angle_threshold_deg =
    node_->declare_parameter<double>("goal_angle_threshold_deg", 10.0);
  param.enable_correct_goal_pose =
    node_->declare_parameter<bool>("enable_correct_goal_pose", true);
  param.consider_no_drivable_lanes =
    node_->declare_parameter<bool>("consider_no_drivable_lanes", false);
  param.check_footprint_inside_lanes =
    node_->declare_parameter<bool>("check_footprint_inside_lanes", true);

  core_ = std::make_shared<DefaultPlannerCore>(param, vehicle_info);
}

void DefaultPlanner::initialize(rclcpp::Node * node)
{
  initializeCommon(node);

  // 벡터맵 subscribe (기존 Autoware topic 과 동일하게)
  map_sub_ = node_->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&DefaultPlanner::onMap, this, std::placeholders::_1));
}

void DefaultPlanner::initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg)
{
  initializeCommon(node);
  // 오프라인/테스트용: 즉시 맵 세팅
  onMap(msg);
}

bool DefaultPlanner::ready() const
{
  return core_ && core_->ready();
}

void DefaultPlanner::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  if (!core_) {
    RCLCPP_WARN(node_->get_logger(), "DefaultPlannerCore is not initialized.");
    return;
  }

  core_->setMap(*msg);
}

DefaultPlanner::LaneletRoute DefaultPlanner::plan(const RoutePoints & points)
{
  if (!core_) {
    RCLCPP_WARN(node_->get_logger(), "DefaultPlannerCore is not initialized.");
    return LaneletRoute{};
  }

  // Core 에게 경로계획 위임
  auto route = core_->plan(points);

  if (route.segments.empty()) {
    RCLCPP_WARN(node_->get_logger(), "DefaultPlannerCore failed to plan route.");
  } else {
    RCLCPP_DEBUG(
      node_->get_logger(), "DefaultPlannerCore planned route: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
      route.start_pose.position.x, route.start_pose.position.y,
      route.goal_pose.position.x, route.goal_pose.position.y);
  }

  // goal footprint 디버그 marker 발행 (있으면)
  auto markers = makeGoalFootprintMarkers(points);
  if (!markers.markers.empty()) {
    pub_goal_footprint_marker_->publish(markers);
  }

  return route;
}

void DefaultPlanner::updateRoute(const LaneletRoute & route)
{
  if (!core_) {
    RCLCPP_WARN(node_->get_logger(), "DefaultPlannerCore is not initialized.");
    return;
  }
  core_->updateRoute(route);
}

void DefaultPlanner::clearRoute()
{
  if (!core_) {
    return;
  }
  core_->clearRoute();
}

MarkerArray DefaultPlanner::visualize(const RoutePoints & points)
{
  // 간단히 goal footprint marker 만 반환 (RViz 디버그용)
  return makeGoalFootprintMarkers(points);
}

MarkerArray DefaultPlanner::makeGoalFootprintMarkers(const RoutePoints & points)
{
  MarkerArray msg;

  if (!node_) {
    return msg;
  }
  if (points.empty()) {
    return msg;
  }

  // 마지막 체크포인트를 goal 로 가정
  const auto & goal = points.back();

  // 차량 footprint 생성 (base_link 기준)
  auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
  const auto local_vehicle_footprint = vehicle_info.createFootprint();

  // goal pose 로 변환
  LinearRing2d goal_footprint =
    transformVector<LinearRing2d>(local_vehicle_footprint, pose2transform(goal));

  // Marker 생성
  auto marker = createDefaultMarker(
    "map", node_->now(), "goal_footprint", 0,
    visualization_msgs::msg::Marker::LINE_STRIP,
    createMarkerScale(0.05, 0.0, 0.0),
    createMarkerColor(0.99, 0.99, 0.2, 1.0));

  // 2.5초 동안 유지
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  // footprint 점들을 순서대로 연결 (마지막에 첫 점 다시 push 해서 폐곡선)
  for (const auto & p : goal_footprint) {
    marker.points.push_back(createPoint(p.x(), p.y(), goal.position.z));
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  msg.markers.push_back(marker);
  return msg;
}

}  // namespace autoware::mission_planner_universe::lanelet2

// pluginlib export (기존 DefaultPlanner 플러그인 이름 유지)
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  autoware::mission_planner_universe::lanelet2::DefaultPlanner,
  autoware::mission_planner_universe::PlannerPlugin)

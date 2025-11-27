#include "default_planner_core.hpp"

#include "utility_functions.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>

#include <tf2/utils.h>

#include <limits>

namespace autoware::mission_planner_universe::lanelet2
{

using autoware::universe_utils::deg2rad;
using autoware::universe_utils::normalizeRadian;
using autoware::universe_utils::pose2transform;
using autoware::universe_utils::transformVector;

DefaultPlannerCore::DefaultPlannerCore(
  const DefaultPlannerParam & param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
: param_(param), vehicle_info_(vehicle_info)
{
}

void DefaultPlannerCore::setMap(const LaneletMapBin & msg)
{
  route_handler_.setMap(msg);
  is_graph_ready_ = true;
}

lanelet::ConstLanelets DefaultPlannerCore::nextLaneletsUpTo(
  const lanelet::ConstLanelet & start_lanelet,
  const double up_to_distance,
  const autoware::route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets result;

  if (up_to_distance <= 0.0) {
    return result;
  }

  const auto nexts = route_handler.getNextLanelets(start_lanelet);
  for (const auto & next : nexts) {
    result.push_back(next);

    const double length = lanelet::geometry::length2d(next);
    const double remain = up_to_distance - length;
    if (remain <= 0.0) {
      continue;
    }

    const auto child = nextLaneletsUpTo(next, remain, route_handler);
    result.insert(result.end(), child.begin(), child.end());
  }

  return result;
}

bool DefaultPlannerCore::checkGoalFootprintInsideLanes(
  const lanelet::ConstLanelet & closest_lanelet_to_goal,
  const lanelet::ConstLanelets & path_lanelets,
  const Polygon2d & goal_footprint) const
{
  MultiPolygon2d lane_polygons;
  Polygon2d tmp_polygon;

  // 1) path 상의 lanelets 및 shoulder lanelets 를 모두 polygon 으로 수집
  for (const auto & ll : path_lanelets) {
    const auto left_shoulder  = route_handler_.getLeftShoulderLanelet(ll);
    const auto right_shoulder = route_handler_.getRightShoulderLanelet(ll);

    if (left_shoulder) {
      boost::geometry::convert(left_shoulder->polygon2d().basicPolygon(), tmp_polygon);
      boost::geometry::correct(tmp_polygon);
      lane_polygons.push_back(tmp_polygon);
    }
    if (right_shoulder) {
      boost::geometry::convert(right_shoulder->polygon2d().basicPolygon(), tmp_polygon);
      boost::geometry::correct(tmp_polygon);
      lane_polygons.push_back(tmp_polygon);
    }

    boost::geometry::convert(ll.polygon2d().basicPolygon(), tmp_polygon);
    boost::geometry::correct(tmp_polygon);
    lane_polygons.push_back(tmp_polygon);
  }

  // 2) goal 근처에서 앞으로 일정 거리까지의 lanelets 도 포함
  const auto extra_lanelets = nextLaneletsUpTo(
    closest_lanelet_to_goal, vehicle_info_.max_longitudinal_offset_m, route_handler_);
  for (const auto & ll : extra_lanelets) {
    boost::geometry::convert(ll.polygon2d().basicPolygon(), tmp_polygon);
    boost::geometry::correct(tmp_polygon);
    lane_polygons.push_back(tmp_polygon);
  }

  // 3) goal footprint 와 lane polygons 의 차집합이 비어 있으면, footprint 가 lane 영역 안에 있다고 판단
  MultiPolygon2d difference;
  boost::geometry::difference(goal_footprint, lane_polygons, difference);

  return boost::geometry::is_empty(difference);
}

bool DefaultPlannerCore::isGoalValid(
  const geometry_msgs::msg::Pose & goal,
  const lanelet::ConstLanelets & path_lanelets) const
{
  const auto & map_ptr = route_handler_.getLaneletMapPtr();
  if (!map_ptr) {
    return false;
  }

  const lanelet::ConstPoint3d goal_point(
    lanelet::InvalId, goal.position.x, goal.position.y, goal.position.z);
  const auto goal_lanelet_point =
    lanelet::utils::conversion::toLaneletPoint(goal.position);

  const double th_angle = deg2rad(param_.goal_angle_threshold_deg);

  // 1) shoulder lanelet 에서 goal 각도/위치를 먼저 확인
  const auto shoulder_lanelets = route_handler_.getShoulderLaneletsAtPose(goal);
  lanelet::Lanelet closest_shoulder_lanelet;
  if (lanelet::utils::query::getClosestLanelet(shoulder_lanelets, goal, &closest_shoulder_lanelet)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // 2) goal 에 가장 가까운 road lanelet 을 찾는다
  lanelet::ConstLanelet closest_lanelet_to_goal;
  const auto road_lanelets_at_goal = route_handler_.getRoadLaneletsAtPose(goal);
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanelets_at_goal, goal, &closest_lanelet_to_goal)) {
    const lanelet::BasicPoint2d goal_p2d(goal.position.x, goal.position.y);
    auto closest_dist = std::numeric_limits<double>::max();

    const auto found = map_ptr->laneletLayer.nearestUntil(
      goal_p2d,
      [&](const auto & bbox, const auto & ll) {
        const auto bbox_dist = lanelet::geometry::distance2d(bbox, goal_p2d);
        if (bbox_dist > closest_dist) {
          return true;  // stop search
        }

        const auto dist = lanelet::geometry::distance2d(goal_p2d, ll.polygon2d());
        if (route_handler_.isRoadLanelet(ll) && dist < closest_dist) {
          closest_dist = dist;
          closest_lanelet_to_goal = ll;
        }
        return false;
      });

    if (!found) {
      return false;
    }
  }

  // 3) 차량 footprint 생성 후 lane/shoulder 영역 밖으로 나가는지 확인
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  LinearRing2d goal_footprint_ring =
    transformVector<LinearRing2d>(local_vehicle_footprint, pose2transform(goal));

  const Polygon2d goal_footprint_polygon =
    convert_linear_ring_to_polygon(goal_footprint_ring);

  if (param_.check_footprint_inside_lanes) {
    const auto parking_lots =
      lanelet::utils::query::getAllParkingLots(map_ptr);
    const bool inside_lanes = checkGoalFootprintInsideLanes(
      closest_lanelet_to_goal, path_lanelets, goal_footprint_polygon);
    const bool inside_parking_lot =
      is_in_parking_lot(parking_lots, goal_lanelet_point);

    if (!inside_lanes && !inside_parking_lot) {
      return false;
    }
  }

  // 4) 일반 lane 내부에 있고, lane 진행 방향과 yaw 차이가 허용 범위 이내면 OK
  if (is_in_lane(closest_lanelet_to_goal, goal_lanelet_point)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_lanelet_to_goal, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // 5) parking space / parking lot 내부면 허용
  const auto parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(map_ptr);
  if (is_in_parking_space(parking_spaces, goal_lanelet_point)) {
    return true;
  }

  const auto parking_lots =
    lanelet::utils::query::getAllParkingLots(map_ptr);
  if (is_in_parking_lot(parking_lots, goal_lanelet_point)) {
    return true;
  }

  return false;
}

LaneletRoute DefaultPlannerCore::plan(const RoutePoints & points)
{
  LaneletRoute route_msg;

  if (!is_graph_ready_) {
    // 맵이 준비되지 않은 경우 빈 route 반환
    return route_msg;
  }

  if (points.size() < 2) {
    // start / goal 최소 2개 포인트 필요
    return route_msg;
  }

  lanelet::ConstLanelets all_route_lanelets;
  RouteSections route_sections;

  // 1) 각 체크포인트 구간마다 lanelet path 를 구해서 이어붙임
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & start_check_point = points.at(i - 1);
    const auto & goal_check_point  = points.at(i);

    lanelet::ConstLanelets path_lanelets;
    const bool ok = route_handler_.planPathLaneletsBetweenCheckpoints(
      start_check_point, goal_check_point, &path_lanelets,
      param_.consider_no_drivable_lanes);
    if (!ok) {
      // 한 구간이라도 실패하면 전체 route 생성 실패로 간주
      return LaneletRoute{};
    }

    // 중복 lanelet(id) 제거하면서 all_route_lanelets 에 append
    for (const auto & lane : path_lanelets) {
      if (!all_route_lanelets.empty() &&
          lane.id() == all_route_lanelets.back().id()) {
        continue;
      }
      all_route_lanelets.push_back(lane);
    }
  }

  if (all_route_lanelets.empty()) {
    return LaneletRoute{};
  }

  // 2) route handler 에 lanelets 세팅 및 route sections 생성
  route_handler_.setRouteLanelets(all_route_lanelets);
  route_sections = route_handler_.createMapSegments(all_route_lanelets);

  if (route_sections.empty()) {
    return LaneletRoute{};
  }

  // 3) goal pose 보정 (centerline 기반)
  auto goal_pose = points.back();
  if (param_.enable_correct_goal_pose) {
    const auto road_lanelets =
      lanelet::utils::query::laneletLayer(route_handler_.getLaneletMapPtr());
    goal_pose = get_closest_centerline_pose(
      road_lanelets, goal_pose, vehicle_info_);
  }

  // 4) goal 유효성 검사 및 route loop 체크
  if (!isGoalValid(goal_pose, all_route_lanelets)) {
    return LaneletRoute{};
  }

  if (autoware::route_handler::RouteHandler::isRouteLooped(route_sections)) {
    return LaneletRoute{};
  }

  // 5) goal z 값(고도) 보정
  const auto refined_goal = refineGoalHeight(goal_pose, route_sections);

  // 6) 최종 LaneletRoute 메시지 구성
  route_msg.start_pose = points.front();
  route_msg.goal_pose  = refined_goal;
  route_msg.segments   = route_sections;

  return route_msg;
}

geometry_msgs::msg::Pose DefaultPlannerCore::refineGoalHeight(
  const geometry_msgs::msg::Pose & goal,
  const RouteSections & route_sections) const
{
  geometry_msgs::msg::Pose refined = goal;

  if (route_sections.empty()) {
    return refined;
  }

  // 마지막 segment 의 preferred_primitive 를 사용해 goal lanelet 추출
  const auto goal_lane_id =
    route_sections.back().preferred_primitive.id;
  const auto goal_lanelet =
    route_handler_.getLaneletsFromId(goal_lane_id);

  const auto goal_lanelet_point =
    lanelet::utils::conversion::toLaneletPoint(goal.position);

  const double z = project_goal_to_map(goal_lanelet, goal_lanelet_point);
  refined.position.z = z;

  return refined;
}

void DefaultPlannerCore::updateRoute(const LaneletRoute & route)
{
  route_handler_.setRoute(route);
}

void DefaultPlannerCore::clearRoute()
{
  route_handler_.clearRoute();
}

}  // namespace autoware::mission_planner_universe::lanelet2

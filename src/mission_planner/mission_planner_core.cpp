#include "mission_planner_core.hpp"

#include "mission_planner.hpp"  // PlannerPlugin, PlannerPlugin::RoutePoints 정의

namespace autoware::mission_planner_universe
{

MissionPlannerCore::MissionPlannerCore(
  const std::string & map_frame,
  const std::shared_ptr<PlannerPlugin> & planner,
  TransformPoseFn transform_pose)
: map_frame_(map_frame), planner_(planner), transform_pose_(std::move(transform_pose))
{
}

LaneletRoute MissionPlannerCore::makeRouteFromSegments(
  const Header & header,
  const std::vector<LaneletSegment> & segments,
  const Pose & start_pose,
  const Pose & goal_pose_in_header_frame,
  const UUID & uuid,
  const bool allow_goal_modification) const
{
  LaneletRoute route;

  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;

  route.start_pose = start_pose;
  route.goal_pose = transform_pose_(goal_pose_in_header_frame, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;

  return route;
}

LaneletRoute MissionPlannerCore::makeRouteFromWaypoints(
  const Header & header,
  const std::vector<Pose> & waypoints_in_header_frame,
  const Pose & start_pose,
  const Pose & goal_pose_in_header_frame,
  const UUID & uuid,
  const bool allow_goal_modification) const
{
  PlannerPlugin::RoutePoints points;
  points.clear();

  // 시작점
  points.push_back(start_pose);

  // header.frame_id 기준으로 들어온 waypoint → map_frame 으로 변환
  for (const auto & wp_header_frame : waypoints_in_header_frame) {
    points.push_back(transform_pose_(wp_header_frame, header));
  }

  // goal 도 동일하게 변환
  points.push_back(transform_pose_(goal_pose_in_header_frame, header));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;

  return route;
}

LaneletRoute MissionPlannerCore::makeRouteForModifiedGoal(
  const PoseWithUuidStamped & msg,
  const LaneletRoute * const current_route,
  const Pose & current_pose) const
{
  const auto & header = msg.header;
  const auto & goal_pose = msg.pose;
  const auto & uuid = msg.uuid;

  // 현재 route가 있으면, 그 route의 allow_modification 을 유지
  const bool allow_goal_modification =
    current_route ? current_route->allow_modification : true;

  // 기존 route가 있으면 그 start_pose 를 사용, 없으면 ego pose 사용
  const Pose & start_pose =
    (current_route != nullptr) ? current_route->start_pose : current_pose;

  std::vector<Pose> waypoints;
  waypoints.clear();

  // 기존 route가 있으면 ego pose 를 첫 waypoint로 사용
  if (current_route != nullptr) {
    waypoints.push_back(current_pose);
  }

  return makeRouteFromWaypoints(
    header, waypoints, start_pose, goal_pose, uuid, allow_goal_modification);
}

}  // namespace autoware::mission_planner_universe

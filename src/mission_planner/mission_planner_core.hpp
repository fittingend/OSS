#pragma once

#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{

using autoware_adapi_v1_msgs::srv::SetRoute;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using geometry_msgs::msg::Pose;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;

// mission_planner.hpp 안에 정의된 플러그인 인터페이스
class PlannerPlugin;

class MissionPlannerCore
{
public:
  // ROS TF 대신, Node 쪽 transform_pose를 콜백으로 받는다.
  using TransformPoseFn = std::function<Pose(const Pose &, const Header &)>;

  MissionPlannerCore(
    const std::string & map_frame,
    const std::shared_ptr<PlannerPlugin> & planner,
    TransformPoseFn transform_pose);

  // SetLaneletRoute 대응: lanelet segment 기반 route 생성
  LaneletRoute makeRouteFromSegments(
    const Header & header,
    const std::vector<LaneletSegment> & segments,
    const Pose & start_pose,
    const Pose & goal_pose_in_header_frame,
    const UUID & uuid,
    bool allow_goal_modification) const;

  // SetWaypointRoute 대응: waypoint 기반 route 생성
  LaneletRoute makeRouteFromWaypoints(
    const Header & header,
    const std::vector<Pose> & waypoints_in_header_frame,
    const Pose & start_pose,
    const Pose & goal_pose_in_header_frame,
    const UUID & uuid,
    bool allow_goal_modification) const;

  // modified goal 기반 reroute
  LaneletRoute makeRouteForModifiedGoal(
    const PoseWithUuidStamped & msg,
    const LaneletRoute * current_route,
    const Pose & current_pose) const;

private:
  std::string map_frame_;
  std::shared_ptr<PlannerPlugin> planner_;
  TransformPoseFn transform_pose_;
};

}  // namespace autoware::mission_planner_universe

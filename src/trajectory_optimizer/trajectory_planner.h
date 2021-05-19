#pragma once

#include "header.h"
#include "path_planner.h"
#include "speed_planner.h"

namespace PlanningOptimizer {

class TrajectoryPlanner
{
public:
	TrajectoryPlanner();
	~TrajectoryPlanner() {};

	bool Init(const std::shared_ptr<zjlmap::Map>& map_data);
    bool RoutingPlan(const XYZ& routing_start_point_xyz, const XYZ& routing_end_point_xyz);
    bool Process(const std::vector<localPlanner::Box2d>& obstacles, const localPlanner::Box2d& robot,
                 std::vector<localPlanner::TargetNode> targets, std::vector<TrajectoryPoint>& planning_trajectory_data);

	std::shared_ptr<Map> map_data_;
	bool is_first_time_routing = true;
	bool trajectory_estop = false;
	NavigationData navigation_data;
	PathPlanner path_planner_handle;
    SpeedPlanner speed_planner_handle;
    RoadId last_road_id;
	RoadId successor_road_id;

	double max_steer_angle = 8.203;
	double max_steering_wheel_angle = 0.6981;
	double wheel_base = 1.0;
};

} // namespace PlanningOptimizer

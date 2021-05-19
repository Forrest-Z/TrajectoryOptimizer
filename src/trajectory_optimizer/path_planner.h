#pragma once

#include <utility>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <iomanip>
#include <algorithm>
#include "cartesian_frenet_transform.h"
#include "planning_visualization.h"
#include "target_line.h"

#include "obstacle_decider.h"
#include "path_optimizer_problem.h"
#include "trajectory_generator.h"

namespace PlanningOptimizer {

using namespace zjlmap;

class PathPlanner
{
public:
	PathPlanner();
	~PathPlanner() {};

	bool Init(const std::shared_ptr<Map>& map_data);
	bool Process(const localPlanner::Box2d& ego_vehicle, std::vector<localPlanner::TargetNode> targets, ObstacleDecider& obstacle_decider_handle, std::vector<CartesianPathPoint>& planning_path_data);
	bool OptimizePath(const std::array<double, 3>& init_state, const PathBoundary& path_boundary, const TargetLine target_line,
		const std::array<double, 5>& weight, std::vector<double>* x, std::vector<double>* dx, std::vector<double>* ddx);
	bool LaneChangeDecider(const localPlanner::Box2d& ego_vehicle, std::vector<localPlanner::TargetNode> targets,
		ObstacleDecider& obstacle_decider_handle, TargetLine& target_line, PathBoundary& path_boundary);
	bool UpdateSelfLaneTargetLineAndPathBoundary(const RoadS& start_s, const localPlanner::TargetNode& target, LaneId curr_lane_id, LaneId target_lane_id,
		TargetLine& target_line, PathBoundary& path_boundary);
	bool UpdateChangeLaneTargetLineAndPathBoundary(const SLZ& start_point_slz, const LaneId& curr_lane_id, const LaneId& next_lane_id,
		TargetLine& target_line, PathBoundary& path_boundary);
    void StitchingSuccesorLane(const LaneId& query_curr_linkage_lane_id, const LaneId& query_next_linkage_lane_id,
                               const bool& is_find_next_lane_id, const double& remain_need_add_length, TargetLine& target_line,
                               PathBoundary& path_boundary);
    // void CreateLastLaneInfo(const TrajectoryPoint& planning_start_point, const SLZ& start_point_slz, ObstacleDecider& obstacle_decider_handle, TargetLine& target_line, PathBoundary& path_boundary);
	bool LaneBorrowDecider(const SLZ& start_point_slz,
		TargetLine& target_line, PathBoundary& path_boundary, ObstacleDecider& obstacle_decider_handle, bool& need_rerouting);
    bool LaneChangeProcess(const SLZ& start_point_slz, const LaneId& curr_lane_id, const LaneId& next_lane_id, const localPlanner::TargetNode& target,
                           TargetLine& target_line, PathBoundary& path_boundary);
    void SelfLaneDriveProcess(const SLZ& start_point_slz, const LaneId& curr_lane_id, TargetLine& target_line, PathBoundary& path_boundary);
    bool IsUTurnLane(const localPlanner::Box2d& ego_vehicle, LaneId target_line_id);

	// map data 
	std::shared_ptr<Map> map_data_ = nullptr;
	Route routing_result_;
	bool is_init_road_id = false;
	int routing_index_of_curr_lane = 0;
	RoadId last_road_id;
	RoadId successor_road_id;
	bool need_rerouting;
	LaneId last_target_lane_id;

	// path planning config
    double forward_length = 50.0;
    double delta_s = 0.25;
	double lateral_derivative_bound = 2.0;
	double lateral_acc_bound = 0.0;
	double lane_change_dis = 15.0;
	double lane_follow_dis = 5.0;
	int max_iter = 4000;
	std::vector<SLZ> pass_lane;
	bool path_driven_estop;
	bool is_uturn = false;
	double road_begin_heading = 0.0;
	double road_end_heading = 0.0;
	std::shared_ptr<localPlanner::TrajectoryGenerator> tg = nullptr;
	std::vector<localPlanner::Trajectory> bezier_trajectories;
	std::vector<double> lat_samples = { -0.5, 0.0, 0.5 };


	// vehicle param 
	double max_steer_angle = 8.203;
	int steer_ratio = 16;
	double max_steer_angle_rate = 8.55211;
	double wheel_base = 1.0;
	double max_steering_wheel_angle = 0.6981;
	double vehicle_width = 1.0;

};

} // namespace PlanningOptimizer

#include "path_planner.h"

namespace PlanningOptimizer
{
	PathPlanner::PathPlanner() {}

	bool PathPlanner::Init(const std::shared_ptr<Map>& map_data)
	{
		map_data_ = map_data;
		last_target_lane_id.road_id = -1;
		last_target_lane_id.local_id = -1;
		last_target_lane_id.section_idx = 0;
		return true;
	}

	bool PathPlanner::Process(const localPlanner::Box2d& ego_vehicle, std::vector<localPlanner::TargetNode> targets,
		ObstacleDecider& obstacle_decider_handle, std::vector<CartesianPathPoint>& planning_path_data)
	{
		std::cout << "[Debug] Enter path planner process ... " << std::endl;
		if (targets.size() <= 0)
		{
			std::cout << "[ERROR] The decision targets size is 0! " << std::endl;
			return false;
		}

		// process the lange change decider
		TargetLine target_line;
		PathBoundary path_boundary;
		PlanningVisualization planning_visualization_handle(map_data_);
		// planning_visualization_handle.VehiclePositionVisualization(ego_vehicle);

		// Add for temp visualization performance  
		// targets[0] = targets[1];
		/*targets[0].sl.s = 40.0; 
		targets[0].rlsl.s += 25;*/

		if (!LaneChangeDecider(ego_vehicle, targets, obstacle_decider_handle, target_line, path_boundary))
		{
			std::cout << "[ERROR] The lane change decider is failed ! " << std::endl;
			return false;
		}

		// debug target_line_points size with target_line_trace_points size
		if (target_line.target_line_points.size() <= 0 || target_line.target_line_trace_points.size() <= 0)
		{
			std::cout << "[ERROR] The target line is empty! " << std::endl;
			return false;
		}

		std::cout << "[Debug] The target_line_points size: " << target_line.target_line_points.size() << std::endl;
		std::cout << "[Debug] The target_line_trace_points size: " << target_line.target_line_trace_points.size() << std::endl;
		std::cout << "[Debug after update path planner] target_line_points.s : " << target_line.target_line_points[0].s
			<< " l : " << target_line.target_line_points[0].l << " road_id : " << target_line.target_line_points[0].lane_id.road_id
			<< " local_id : " << target_line.target_line_points[0].lane_id.local_id << std::endl;
		std::cout << "[Debug after update path planner] Lane-based path boundary s : " << path_boundary.road_based_sl_boundary[0].first.s
			<< " left l : " << path_boundary.lane_based_sl_boundary[0].first
			<< " right l : " << path_boundary.lane_based_sl_boundary[0].second << std::endl;
		std::cout << "[Debug after update path planner] Road-based path boundary s : " << path_boundary.road_based_sl_boundary[0].first.s
			<< " left l : " << path_boundary.road_based_sl_boundary[0].first.l
			<< " right l : " << path_boundary.road_based_sl_boundary[0].second.l << std::endl;

		// Use bezier method to deal u-turn situation 
		if (IsUTurnLane(ego_vehicle, ego_vehicle.rlsl.lane_id))
		{
			bezier_trajectories.clear(); 
			tg = std::make_shared<localPlanner::TrajectoryGenerator>(lat_samples);
			bezier_trajectories = tg->CandidateTrajectoryGenerator(ego_vehicle, targets);
			auto comp = [](localPlanner::Trajectory first, localPlanner::Trajectory second) {
				return first.target.driving_prob > second.target.driving_prob;
			};
			std::sort(bezier_trajectories.begin(), bezier_trajectories.end(), comp);

			std::cout << "[Debug] The bezier trajectory size: " << bezier_trajectories.size() << std::endl; 

			double accumulated_dis = 0.0;
			for (size_t i = 0; i < bezier_trajectories[0].trajectory.poses.size(); i++)
			{
				CartesianPathPoint optimized_cartesian_path_point;
				optimized_cartesian_path_point.x = bezier_trajectories[0].trajectory.poses[i].x;
				optimized_cartesian_path_point.y = bezier_trajectories[0].trajectory.poses[i].y;
				optimized_cartesian_path_point.z = 0.0;

				// calculate theta
				optimized_cartesian_path_point.theta = bezier_trajectories[0].trajectory.poses[i].theta;

				// calculate kappa
				optimized_cartesian_path_point.kappa = bezier_trajectories[0].trajectory.poses[i].kappa;

				optimized_cartesian_path_point.accumulate_dis = accumulated_dis;
				accumulated_dis += delta_s;


				planning_path_data.push_back(optimized_cartesian_path_point);
			}

			// planning_visualization_handle.SLZVectorVisualization(target_line.target_line_points);
			// planning_visualization_handle.PathBoundaryVisualization(path_boundary.road_based_sl_boundary);
			// planning_visualization_handle.PathFeatureVisualization(planning_path_data, "theta");
			// planning_visualization_handle.PathFeatureVisualization(planning_path_data, "kappa");
			// planning_visualization_handle.CartesianPathVisualization(planning_path_data);
			// planning_visualization_handle.PointVisualization(targets[0].pose);

			return true;
		}
		//

		TracePoint target_trace_point;
		target_line.GetTargetTracePoint(ego_vehicle.rlsl, target_trace_point);
		std::cout << "[Debug] target trace point s: " << target_trace_point.s << " l : " << target_trace_point.l << std::endl;

		// get the init point s_condition & l_condition
		std::pair<std::array<double, 3>, std::array<double, 3>> init_frenet_state;

		// cartesian_to_frenet
		CartesianFrenetTransform::CartesianToFrenet(ego_vehicle, target_trace_point, init_frenet_state);
		std::cout << "init frenet frame s : " << init_frenet_state.first[0] << std::endl;
		std::cout << "init frenet frame ds : " << init_frenet_state.first[1] << std::endl;
		std::cout << "init frenet frame dds : " << init_frenet_state.first[2] << std::endl;
		std::cout << "init frenet frame l : " << init_frenet_state.second[0] << std::endl;
		std::cout << "init frenet frame dl : " << init_frenet_state.second[1] << std::endl;
		std::cout << "init frenet frame ddl : " << init_frenet_state.second[2] << std::endl;

		// construct the optimization problem
		// std::array<double, 5> weight = { 1.0, 20.0, 1000.0, 50000.0, 0.0 }; // default path config
		// std::array<double, 5> weight = { 40.0, 50.0, 400.0, 3000.0, 0.0 };  // 12.7 config
		std::array<double, 5> weight = { 40.0, 40.0, 200.0, 1000.0, 0.0 }; 

		/*
		// diff weight for self-lane situation
		if (target_line.target_line_type == TargetLineType::self_lane)
		{
			weight = { 40.0, 50.0, 800.0, 20000.0, 0.0 };
		}
		else if (target_line.target_line_type == TargetLineType::lane_change)
		{
			weight = { 40.0, 50.0, 400.0, 3000.0, 0.0 };
		}
		*/

		std::vector<double> optimized_path_l;
		std::vector<double> optimized_path_dl;
		std::vector<double> optimized_path_ddl;

		// planning_visualization_handle.SLZVectorVisualization(target_line.target_line_points);
		// planning_visualization_handle.PathBoundaryVisualization(path_boundary.road_based_sl_boundary);

		bool optimize_status = OptimizePath(init_frenet_state.second, path_boundary, target_line, weight, &optimized_path_l, &optimized_path_dl,
			&optimized_path_ddl);
		if (!optimize_status)
		{
			std::cout << "[ERROR] path planner Optimize path step is failed ! ";
			return false;
		}

		// // debug optimized path
		// std::cout << "[Debug] optimized path l size: " << optimized_path_l.size();
		// std::cout << "[Debug] optimized_path_l : ";
		// for (size_t i = 0; i < optimized_path_l.size(); i++)
		// {
		// 	std::cout << optimized_path_l[i] << " ";
		// }
		// std::cout;
		// std::cout << "[Debug] optimized_path_dl : ";
		// for (size_t i = 0; i < optimized_path_dl.size(); i++)
		// {
		// 	std::cout << optimized_path_dl[i] << " ";
		// }
		// std::cout;
		// std::cout << "[Debug] optimized_path_ddl : ";
		// for (size_t i = 0; i < optimized_path_l.size(); i++)
		// {
		// 	std::cout << optimized_path_ddl[i] << " ";
		// }
		// std::cout;

		// convert optimized lane-based SL path to cartesian path
		// std::vector<zjlmap::SLZ> optimized_frenet_path = target_line.target_line_points;
		std::vector<FrenetFramePathPoint> optimized_frenet_path;
		optimized_frenet_path.resize(target_line.target_line_points.size());
		for (size_t i = 0; i < optimized_frenet_path.size(); i++)
		{
			optimized_frenet_path[i].lane_id = target_line.target_line_points[i].lane_id;
			optimized_frenet_path[i].s = target_line.target_line_points[i].s;
			optimized_frenet_path[i].z = target_line.target_line_points[i].z;
			int sign = target_line.target_line_points[i].l < 0 ? -1 : 1;
			optimized_frenet_path[i].l = target_line.target_line_points[i].l + sign * optimized_path_l[i];  // convert to road-based SL frame
			optimized_frenet_path[i].dl = optimized_path_dl[i];
			optimized_frenet_path[i].ddl = optimized_path_ddl[i];
		}

		double accumulated_length = 0.0;
		for (size_t i = 0; i < optimized_frenet_path.size(); i++)
		{
			// calculate xyz
			SLZ optimized_path_point_slz =
				SLZ(optimized_frenet_path[i].lane_id, optimized_frenet_path[i].s, optimized_frenet_path[i].l, optimized_frenet_path[i].z);
			XYZ optimized_path_point_xyz = map_data_->xyz(optimized_path_point_slz);
			CartesianPathPoint optimized_cartesian_path_point;
			optimized_cartesian_path_point.x = optimized_path_point_xyz.x;
			optimized_cartesian_path_point.y = optimized_path_point_xyz.y;
			optimized_cartesian_path_point.z = optimized_path_point_xyz.z;

			// calculate theta
			TracePoint ref_trace_point;
			target_line.GetTargetTracePoint(optimized_path_point_slz, ref_trace_point);
			optimized_cartesian_path_point.theta =
				CartesianFrenetTransform::CalculateTheta(ref_trace_point.hdg, ref_trace_point.curv, optimized_path_l[i], optimized_path_dl[i]);

			// calculate kappa
			optimized_cartesian_path_point.kappa = -CartesianFrenetTransform::CalculateKappa(
				-ref_trace_point.curv, -ref_trace_point.curv_deriv, optimized_path_l[i], optimized_path_dl[i], optimized_path_ddl[i]);

			// calculate planning path length
			optimized_cartesian_path_point.accumulate_dis = accumulated_length;
			accumulated_length += delta_s;

			optimized_cartesian_path_point.lane_id = optimized_path_point_slz.lane_id;
			optimized_cartesian_path_point.s = optimized_path_point_slz.s;

			

			planning_path_data.push_back(optimized_cartesian_path_point);
		}

		// planning_visualization_handle.FrenetPathVisualization(optimized_path_l, optimized_path_dl, optimized_path_ddl);
		// planning_visualization_handle.PathFeatureVisualization(planning_path_data, "theta");
		// planning_visualization_handle.PathFeatureVisualization(planning_path_data, "kappa");
		// planning_visualization_handle.CartesianPathVisualization(planning_path_data);
		// planning_visualization_handle.PassLaneVisualization(pass_lane);
		// planning_visualization_handle.PointVisualization(targets[0].pose);

		return true;
	}

	bool PathPlanner::LaneChangeDecider(const localPlanner::Box2d& ego_vehicle, std::vector<localPlanner::TargetNode> targets,
		ObstacleDecider& obstacle_decider_handle, TargetLine& target_line, PathBoundary& path_boundary)
	{
		std::cout << "[Debug] ego_vehicle s: " << ego_vehicle.rlsl.s << " l: " << ego_vehicle.rlsl.l
			<< " road_id: " << ego_vehicle.rlsl.lane_id.road_id << " section_idx: " << ego_vehicle.rlsl.lane_id.section_idx
			<< " local_id: " << ego_vehicle.rlsl.lane_id.local_id << std::endl;

		// Find the curr_lane_id and target_lane_id
		LaneId curr_lane_id = ego_vehicle.rlsl.lane_id;
		LaneId target_lane_id = targets[0].rlsl.lane_id;
		RoadS start_s = ego_vehicle.rlsl.s;
		std::cout << "[Debug] curr_lane_id road_id: " << curr_lane_id.road_id << " section_idx: " << curr_lane_id.section_idx
			<< " local_id: " << curr_lane_id.local_id << std::endl;
		std::cout << "[Debug] target_lane_id road_id: " << target_lane_id.road_id << " section_idx: " << target_lane_id.section_idx
			<< " local_id: " << target_lane_id.local_id << " target length: " << targets[0].sl.s << std::endl;

		// decide if need change lane according to behavior planner
		if (targets[0].lane_type == localPlanner::LaneType::LEFT ||
			targets[0].lane_type == localPlanner::LaneType::RIGHT)  // If need change lane
		{
			std::cout << "[Debug] Enter Lane Change situation. " << std::endl;
			if (!LaneChangeProcess(ego_vehicle.rlsl, curr_lane_id, target_lane_id, targets[0], target_line, path_boundary))
			{
				std::cout << "[ERROR] The lane change process is failed! " << std::endl; 
				return false; 
			}
			obstacle_decider_handle.ProcessLaneChangeObstacles(ego_vehicle, targets, target_line, path_boundary);
		}
		else if (targets[0].lane_type == localPlanner::LaneType::MIDDLE)  // If keep self lane
		{
			std::cout << "[Debug] Enter Self Lane Follow situation. " << std::endl;
			// SelfLaneDriveProcess(ego_vehicle.rlsl, curr_lane_id, target_line, path_boundary);

			// Update self lane target_line and path_boundary
			if (!UpdateSelfLaneTargetLineAndPathBoundary(start_s, targets[0], curr_lane_id, target_lane_id, target_line, path_boundary))
			{
				std::cout << "[ERROR path planner] The update self lane target line process is failed ! " << std::endl;
				return false;
			}
			std::cout << "[Debug] After entering the update target_line size: " << target_line.target_line_points.size() << std::endl;

			// shrink the path boundary with one half vehicle width for self-lane situation
			for (size_t i = static_cast<int>(lane_follow_dis / delta_s); i < path_boundary.lane_based_sl_boundary.size(); i++)
			{
				path_boundary.lane_based_sl_boundary[i].first += vehicle_width / 2.0;
				path_boundary.lane_based_sl_boundary[i].second -= vehicle_width / 2.0;
			}

			if (target_line.target_line_points.size() != path_boundary.lane_based_sl_boundary.size())
			{
				std::cout << "[ERROR path planner] the target_line size isn't equal to the path_boundary size !" << std::endl;
				std::cout << "[Debug] target_line_points.size : " << target_line.target_line_points.size()
					<< " path_boundary.boundary.size :  " << path_boundary.lane_based_sl_boundary.size() << std::endl;
				return false;
			}
		}
		return true;
	}

	void PathPlanner::StitchingSuccesorLane(const LaneId& query_curr_linkage_lane_id, const LaneId& query_next_linkage_lane_id,
		const bool& is_find_next_lane_id, const double& remain_need_add_length, TargetLine& target_line,
		PathBoundary& path_boundary)
	{
		LaneLinkage curr_lane_linkage;
		map_data_->query_lane_linkage(query_curr_linkage_lane_id, curr_lane_linkage);
		for (size_t i = 0; i < curr_lane_linkage.successor_lanes.size(); i++)
		{
			std::cout << "[Debug] curr lane successor " << i << " road_id: " << curr_lane_linkage.successor_lanes[i].road_id
				<< " local id: " << curr_lane_linkage.successor_lanes[i].local_id << std::endl;
		}

		LaneId successor_lane;
		if (curr_lane_linkage.successor_lanes.size() == 0)
		{
			std::cout << "[Debug path planner] The size of curr lane's successor lanes is 0! " << std::endl;
			return;
		}
		else if (curr_lane_linkage.successor_lanes.size() == 1)
		{
			successor_lane = curr_lane_linkage.successor_lanes[0];
		}
		else
		{
			if (is_find_next_lane_id)
			{
				for (size_t i = 0; i < curr_lane_linkage.successor_lanes.size(); i++)
				{
					if (curr_lane_linkage.successor_lanes[i].road_id ==
						query_next_linkage_lane_id.road_id)  // pick the first lane which has the same road_id as next_lane
					{
						successor_lane = curr_lane_linkage.successor_lanes[i];
						break;
					}
				}
			}
			else
			{
				successor_lane =
					curr_lane_linkage.successor_lanes[0];  // successor lanes don't match any routing segment, so just pick the first one
			}
		}

		std::cout << "[Debug] successor lane road_id: " << successor_lane.road_id << " local id: " << successor_lane.local_id << std::endl;

		// get the successor lane info
		successor_road_id = successor_lane.road_id;
		LaneInfo successor_lane_info = map_data_->query_lane_info(successor_lane);
		SLZArray successor_lane_slz_array = EmptySLZArray;
		std::vector<TracePoint> successor_lane_trace_points;
		map_data_->calc_lane_center_line(successor_lane, successor_lane_info.begin, successor_lane_info.end, delta_s, successor_lane_slz_array);
		map_data_->calc_lane_center_line_curv(successor_lane, successor_lane_info.begin, successor_lane_info.end, delta_s,
			successor_lane_trace_points);

		SLZArray left_successor_lane_boundary = EmptySLZArray;
		SLZArray right_successor_lane_boundary = EmptySLZArray;
		map_data_->query_lane_boundaries(successor_lane, delta_s, left_successor_lane_boundary, right_successor_lane_boundary);

		// double curr_add_length = successor_lane_info.end;
		uint32_t successor_lane_add_size = fabs(successor_lane_info.end - successor_lane_info.begin) > remain_need_add_length
			? static_cast<uint32_t>(remain_need_add_length / delta_s)
			: successor_lane_slz_array.length;
		for (uint32_t i = 0; i < successor_lane_add_size; i++)
		{
			target_line.target_line_points.push_back(successor_lane_slz_array[i]);
			target_line.target_line_trace_points.push_back(successor_lane_trace_points[i]);
			path_boundary.lane_based_sl_boundary.push_back(
				std::make_pair(-abs(left_successor_lane_boundary.slz_array[i].l - successor_lane_slz_array[i].l),
					abs(right_successor_lane_boundary.slz_array[i].l - successor_lane_slz_array[i].l)));
			path_boundary.road_based_sl_boundary.push_back(
				std::make_pair(left_successor_lane_boundary.slz_array[i], right_successor_lane_boundary.slz_array[i]));
		}

		return;
	}

	bool PathPlanner::LaneChangeProcess(const SLZ& start_point_slz, const LaneId& curr_lane_id, const LaneId& next_lane_id,
		const localPlanner::TargetNode& target, TargetLine& target_line, PathBoundary& path_boundary)
	{
		// decide if target is too close to the ego vehicle 
		double distance_to_ego = fabs(target.rlsl.s - start_point_slz.s);
		if (distance_to_ego < 10.0)
		{
			std::cout << "[ERROR] The target is too close to ego vehicle for lane change! " << std::endl; 
			return false;
		}


		target_line.target_line_type = TargetLineType::lane_change;
		// LaneInfo next_lane_info = map_data_->query_lane_info(next_lane_id);
		SLZArray target_line_slz_array = EmptySLZArray;
		map_data_->calc_lane_center_line(next_lane_id, start_point_slz.s, target.rlsl.s, delta_s, target_line_slz_array);
		map_data_->calc_lane_center_line_curv(next_lane_id, start_point_slz.s, target.rlsl.s, delta_s, target_line.target_line_trace_points);
		for (uint32_t i = 0; i < target_line_slz_array.length; i++)
		{
			target_line.target_line_points.push_back(target_line_slz_array.slz_array[i]);
		}

		SLZArray next_lane_left_path_boundary = EmptySLZArray;
		SLZArray next_lane_right_path_boundary = EmptySLZArray;
		map_data_->query_lane_boundaries(next_lane_id, start_point_slz.s, target.rlsl.s, delta_s, next_lane_left_path_boundary,
			next_lane_right_path_boundary);

		path_boundary.start_s = start_point_slz.s;
		for (uint32_t i = 0; i < next_lane_left_path_boundary.length; i++)
		{
			path_boundary.lane_based_sl_boundary.push_back(
				std::make_pair(-abs(next_lane_left_path_boundary.slz_array[i].l - target_line.target_line_points[i].l),
					abs(next_lane_right_path_boundary.slz_array[i].l - target_line.target_line_points[i].l)));
			path_boundary.road_based_sl_boundary.push_back(
				std::make_pair(next_lane_left_path_boundary.slz_array[i], next_lane_right_path_boundary.slz_array[i]));
			SLZ pass_lane_point = abs(next_lane_id.local_id) > abs(curr_lane_id.local_id) ? next_lane_left_path_boundary.slz_array[i]
				: next_lane_right_path_boundary.slz_array[i];
			pass_lane.push_back(pass_lane_point);
		}

		// Comment for temp 
		/*// Update change lane target_line and path_boundary
		if (!UpdateChangeLaneTargetLineAndPathBoundary(start_point_slz, curr_lane_id, next_lane_id, target_line, path_boundary))
		{
			std::cout << "[ERROR path planner] The update change lane target line process is failed ! ";
			return false;
		}*/

		double curr_lane_width = 0.0;
		map_data_->query_lane_width_at(start_point_slz, curr_lane_width);
		std::cout << "[Debug] curr lane width: " << curr_lane_width << std::endl;
		// extend the path boundary
		if (abs(next_lane_id.local_id) > abs(curr_lane_id.local_id))  // right lane change
		{

			// for (size_t i = 0; i < static_cast<size_t>(lane_change_dis / delta_s); i++)
			// for (size_t i = 0; i < path_boundary.road_based_sl_boundary.size(); i++)
			for (size_t i = 0; i < static_cast<size_t>(lane_change_dis / delta_s); i++)
			{
				/*
				LaneChangeType right_lane_change_type = map_data_->query_lane_change_type_at(next_lane_id, start_point_slz.s);
				if (right_lane_change_type == LaneChangeType::kBothChange || right_lane_change_type == LaneChangeType::kLeftChange)
				{
					std::cout << "[Debug] Enter left changable situation. ";
					if (path_boundary.road_based_sl_boundary[i].second.l > 0)
					{
						path_boundary.road_based_sl_boundary[i].first.l -= curr_lane_width;
						path_boundary.lane_based_sl_boundary[i].first -= curr_lane_width;
					}
					else
					{
						path_boundary.road_based_sl_boundary[i].first.l += curr_lane_width;
						path_boundary.lane_based_sl_boundary[i].first -= curr_lane_width;
					}
				}
				*/
				if (path_boundary.road_based_sl_boundary[i].second.l > 0)
				{
					path_boundary.road_based_sl_boundary[i].first.l -= curr_lane_width;
					path_boundary.lane_based_sl_boundary[i].first -= curr_lane_width;
				}
				else
				{
					path_boundary.road_based_sl_boundary[i].first.l += curr_lane_width;
					path_boundary.lane_based_sl_boundary[i].first -= curr_lane_width;
				}
			}
		}
		else  // left lane change
		{
			// for (size_t i = 0; i < static_cast<size_t>(lane_change_dis / delta_s); i++)
			// for (size_t i = 0; i < path_boundary.road_based_sl_boundary.size(); i++)
			for (size_t i = 0; i < static_cast<size_t>(lane_change_dis / delta_s); i++)
			{
				if (path_boundary.road_based_sl_boundary[i].second.l > 0)
				{
					path_boundary.road_based_sl_boundary[i].second.l += curr_lane_width;
					path_boundary.lane_based_sl_boundary[i].second += curr_lane_width;
				}
				else
				{
					path_boundary.road_based_sl_boundary[i].second.l -= curr_lane_width;
					path_boundary.lane_based_sl_boundary[i].second += curr_lane_width;
				}
				//LaneChangeType left_lane_change_type = map_data_->query_lane_change_type_at(next_lane_id, start_point_slz.s);
				//if (left_lane_change_type == LaneChangeType::kBothChange || left_lane_change_type == LaneChangeType::kRightChange)
				//{
				//	// std::cout << "[Debug] Enter right changable situation. ";
				//	if (path_boundary.road_based_sl_boundary[i].second.l > 0)
				//	{
				//		path_boundary.road_based_sl_boundary[i].second.l += curr_lane_width;
				//		path_boundary.lane_based_sl_boundary[i].second += curr_lane_width;
				//	}
				//	else
				//	{
				//		path_boundary.road_based_sl_boundary[i].second.l -= curr_lane_width;
				//		path_boundary.lane_based_sl_boundary[i].second += curr_lane_width;
				//	}
				//}
			}
		}

		return true;
	}

	bool PathPlanner::UpdateChangeLaneTargetLineAndPathBoundary(const SLZ& start_point_slz, const LaneId& curr_lane_id,
		const LaneId& next_lane_id, TargetLine& target_line, PathBoundary& path_boundary)
	{
		// if the remain length of current lane is greater than forward_length, then truncate
		double target_line_end_s = target_line.target_line_points.back().s;
		std::cout << "[debug] target line end s : " << target_line_end_s << std::endl;
		if (abs(target_line_end_s - start_point_slz.s) > forward_length)
		{
			// truncate the target_line and path boundary
			double truncate_end_s =
				target_line_end_s > start_point_slz.s ? start_point_slz.s + forward_length : start_point_slz.s - forward_length;
			for (uint32_t i = 0; i < static_cast<size_t>(abs(truncate_end_s - target_line_end_s) / delta_s); i++)
			{
				target_line.target_line_points.pop_back();
				target_line.target_line_trace_points.pop_back();
				path_boundary.lane_based_sl_boundary.pop_back();
				path_boundary.road_based_sl_boundary.pop_back();
				pass_lane.pop_back();
			}
			return true;
		}

		// else add successor lane to target_line
		double remain_need_add_length = forward_length - abs(target_line_end_s - start_point_slz.s);
		LaneId query_curr_linkage_lane_id = next_lane_id;
		LaneId query_next_linkage_lane_id;
		bool is_find_next_lane_id = false;
		std::cout << "[Debug] routing index of curr lane : " << routing_index_of_curr_lane << std::endl;
		for (size_t i = routing_index_of_curr_lane + 1; i < routing_result_.lane_id_vec.size(); i++)
		{
			LaneId routing_segment = routing_result_.lane_id_vec[i];
			if (next_lane_id.road_id != routing_segment.road_id)
			{
				is_find_next_lane_id = true;
				query_next_linkage_lane_id = routing_segment;
				break;
			}
		}
		std::cout << "[Debug] is find next lane id: " << is_find_next_lane_id << std::endl;
		std::cout << "[Debug] query next linkage lane road id: " << query_next_linkage_lane_id.road_id
			<< " local id: " << query_next_linkage_lane_id.local_id << std::endl;

		StitchingSuccesorLane(query_curr_linkage_lane_id, query_next_linkage_lane_id, is_find_next_lane_id, remain_need_add_length, target_line,
			path_boundary);

		return true;
	}

	void PathPlanner::SelfLaneDriveProcess(const SLZ& start_point_slz, const LaneId& curr_lane_id, TargetLine& target_line,
		PathBoundary& path_boundary)
	{
		target_line.target_line_type = TargetLineType::self_lane;
		LaneInfo curr_lane_info = map_data_->query_lane_info(curr_lane_id);
		SLZArray target_line_slz_array = EmptySLZArray;
		map_data_->calc_lane_center_line(curr_lane_id, start_point_slz.s, curr_lane_info.end, delta_s, target_line_slz_array);
		map_data_->calc_lane_center_line_curv(curr_lane_id, start_point_slz.s, curr_lane_info.end, delta_s,
			target_line.target_line_trace_points);
		for (uint32_t i = 0; i < target_line_slz_array.length; i++)
		{
			target_line.target_line_points.push_back(target_line_slz_array.slz_array[i]);
		}
		std::cout << "[Debug] Before entering the update target_line size: " << target_line.target_line_points.size() << std::endl;

		SLZArray left_path_boundary = EmptySLZArray;
		SLZArray right_path_boundary = EmptySLZArray;
		map_data_->query_lane_boundaries(curr_lane_id, start_point_slz.s, curr_lane_info.end, delta_s, left_path_boundary, right_path_boundary);
		path_boundary.start_s = start_point_slz.s;

		std::cout << "[Debug] left path boundary length: " << left_path_boundary.length << std::endl;
		for (uint32_t i = 0; i < left_path_boundary.length; i++)
		{
			path_boundary.lane_based_sl_boundary.push_back(
				std::make_pair(-abs(left_path_boundary.slz_array[i].l - target_line.target_line_points[i].l),
					abs(right_path_boundary.slz_array[i].l - target_line.target_line_points[i].l)));
			path_boundary.road_based_sl_boundary.push_back(std::make_pair(left_path_boundary.slz_array[i], right_path_boundary.slz_array[i]));
		}

		return;
	}

	bool PathPlanner::UpdateSelfLaneTargetLineAndPathBoundary(const RoadS& start_s, const localPlanner::TargetNode& target, LaneId curr_lane_id, 
		LaneId target_lane_id, TargetLine& target_line, PathBoundary& path_boundary)
	{
		target_line.target_line_type = TargetLineType::self_lane;

		// if target is on the current lane  
		if (curr_lane_id == target_lane_id)
		{
			// LaneInfo curr_lane_info = map_data_->query_lane_info(curr_lane_id);
			SLZArray target_line_slz_array = EmptySLZArray;
			map_data_->calc_lane_center_line(curr_lane_id, start_s, target.rlsl.s, delta_s, target_line_slz_array);
			map_data_->calc_lane_center_line_curv(curr_lane_id, start_s, target.rlsl.s, delta_s,
				target_line.target_line_trace_points);
			for (uint32_t i = 0; i < target_line_slz_array.length; i++)
			{
				target_line.target_line_points.push_back(target_line_slz_array.slz_array[i]);
			}

			SLZArray left_path_boundary = EmptySLZArray;
			SLZArray right_path_boundary = EmptySLZArray;
			map_data_->query_lane_boundaries(curr_lane_id, start_s, target.rlsl.s, delta_s, left_path_boundary, right_path_boundary);
			path_boundary.start_s = start_s;

			for (uint32_t i = 0; i < left_path_boundary.length; i++)
			{
				path_boundary.lane_based_sl_boundary.push_back(
					std::make_pair(-abs(left_path_boundary.slz_array[i].l - target_line.target_line_points[i].l),
						abs(right_path_boundary.slz_array[i].l - target_line.target_line_points[i].l)));
				path_boundary.road_based_sl_boundary.push_back(std::make_pair(left_path_boundary.slz_array[i], right_path_boundary.slz_array[i]));
			}
		}
		else // target is on the successor lane 
		{
			// Add the rest part of the current lane
			LaneInfo curr_lane_info = map_data_->query_lane_info(curr_lane_id);
			SLZArray target_line_slz_array = EmptySLZArray;
			map_data_->calc_lane_center_line(curr_lane_id, start_s, curr_lane_info.end, delta_s, target_line_slz_array);
			map_data_->calc_lane_center_line_curv(curr_lane_id, start_s, curr_lane_info.end, delta_s,
				target_line.target_line_trace_points);
			for (uint32_t i = 0; i < target_line_slz_array.length; i++)
			{
				target_line.target_line_points.push_back(target_line_slz_array.slz_array[i]);
			}

			SLZArray left_path_boundary = EmptySLZArray;
			SLZArray right_path_boundary = EmptySLZArray;
			map_data_->query_lane_boundaries(curr_lane_id, start_s, curr_lane_info.end, delta_s, left_path_boundary, right_path_boundary);
			path_boundary.start_s = start_s;

			for (uint32_t i = 0; i < left_path_boundary.length; i++)
			{
				path_boundary.lane_based_sl_boundary.push_back(
					std::make_pair(-abs(left_path_boundary.slz_array[i].l - target_line.target_line_points[i].l),
						abs(right_path_boundary.slz_array[i].l - target_line.target_line_points[i].l)));
				path_boundary.road_based_sl_boundary.push_back(std::make_pair(left_path_boundary.slz_array[i], right_path_boundary.slz_array[i]));
			}

			// Add till the target  
			LaneInfo target_lane_info = map_data_->query_lane_info(target_lane_id);
			SLZArray target_line_slz_array_rest = EmptySLZArray; 
			map_data_->calc_lane_center_line(target_lane_id, target_lane_info.begin, target.rlsl.s, delta_s, target_line_slz_array_rest);
			std::vector<zjlmap::TracePoint> target_line_trace_points_rest;
			map_data_->calc_lane_center_line_curv(target_lane_id, target_lane_info.begin, target.rlsl.s, delta_s,
				target_line_trace_points_rest);
			for (uint32_t i = 0; i < target_line_slz_array_rest.length; i++)
			{
				target_line.target_line_points.push_back(target_line_slz_array_rest.slz_array[i]);
				target_line.target_line_trace_points.push_back(target_line_trace_points_rest[i]);
			}
			SLZArray left_path_boundary_rest = EmptySLZArray;
			SLZArray right_path_boundary_rest = EmptySLZArray;
			map_data_->query_lane_boundaries(target_lane_id, target_lane_info.begin, target.rlsl.s, delta_s, left_path_boundary_rest, right_path_boundary_rest);
			for (uint32_t i = 0; i < left_path_boundary_rest.length; i++)
			{
				path_boundary.lane_based_sl_boundary.push_back(
					std::make_pair(-abs(left_path_boundary_rest.slz_array[i].l - target_line_slz_array_rest.slz_array[i].l),
						abs(right_path_boundary_rest.slz_array[i].l - target_line_slz_array_rest.slz_array[i].l)));
				path_boundary.road_based_sl_boundary.push_back(std::make_pair(left_path_boundary_rest.slz_array[i], right_path_boundary_rest.slz_array[i]));
			}

		}

		return true;
	}

	bool PathPlanner::IsUTurnLane(const localPlanner::Box2d& ego_vehicle, LaneId target_line_id)
	{
		if (!(target_line_id == last_target_lane_id))
		{
			LaneInfo target_line_info = map_data_->query_lane_info(target_line_id);
			std::vector<TracePoint> target_line_trace_points;
			SLZArray target_line_slz_array;
			map_data_->calc_lane_center_line(target_line_id, target_line_info.begin, target_line_info.end, delta_s, target_line_slz_array);
			map_data_->calc_lane_center_line_curv(target_line_id, target_line_info.begin, target_line_info.end, delta_s,
				target_line_trace_points);

			map_data_->calc_road_heading_angle(target_line_slz_array.slz_array[0], road_begin_heading);
			map_data_->calc_road_end_heading_angle(target_line_id, road_end_heading);
			double heading_diff = fabs(road_begin_heading - road_end_heading) * 180 / M_PI;
			std::cout << "[Debug] u-turn decider target line road_id: " << target_line_id.road_id
				<< " section_idx: " << target_line_id.section_idx << " local_id: " << target_line_id.local_id << std::endl;
			std::cout << "[Debug] road begin heading: " << road_begin_heading << " road_end_heading: " << road_end_heading
				<< " heading diff: " << heading_diff << std::endl;

			if (heading_diff > 160 && heading_diff < 200)
			{
				is_uturn = true;
			}
			else
			{
				is_uturn = false;
			}
			std::cout << "[Debug] is_uturn1 : " << is_uturn << std::endl;
			last_target_lane_id = target_line_id;
		}
		
		return is_uturn;
	}

	bool PathPlanner::OptimizePath(const std::array<double, 3>& init_state, const PathBoundary& path_boundary, const TargetLine target_line,
		const std::array<double, 5>& weight, std::vector<double>* x, std::vector<double>* dx,
		std::vector<double>* ddx)
	{
		const size_t path_size = path_boundary.lane_based_sl_boundary.size();
		if (path_size <= 2)
		{
			std::cout << "[ERROR] The path boundary size is less than 2! " << std::endl;
			return false;
		}
		std::cout << "[Debug] The path boundary size is: " << path_size << std::endl;
		PathOptimizerProblem path_optimizer(path_size, delta_s, init_state);

		path_optimizer.weight_x_ = weight[0];
		path_optimizer.weight_dx_ = weight[1];
		path_optimizer.weight_ddx_ = weight[2];
		path_optimizer.weight_dddx_ = weight[3];
		path_optimizer.scale_factor_ = { 1.0, 10.0, 100.0 };

		std::cout << "[Debug] the path optimizer value assignee is finished! " << std::endl; 

		// set constraints
		path_optimizer.x_bounds_.resize(path_size);
		for (uint32_t i = 0; i < path_size; i++)
		{
			if (path_boundary.lane_based_sl_boundary[i].first > path_boundary.lane_based_sl_boundary[i].second)
			{
				std::cout << "[ERROR] The left path boundary is greater than right path boundary, cannot get optimized path! " << std::endl;
				return false;
			}
			path_optimizer.x_bounds_[i].first = path_boundary.lane_based_sl_boundary[i].first;
			path_optimizer.x_bounds_[i].second = path_boundary.lane_based_sl_boundary[i].second;
		}

		std::cout << "[Debug] the path optimizer x_bounds_ assignee is finished! " << std::endl; 

		for (uint32_t i = 0; i < path_size; i++)
		{
			path_optimizer.dx_bounds_[i].first = -lateral_derivative_bound;
			path_optimizer.dx_bounds_[i].second = lateral_derivative_bound;
		}
		std::cout << "[Debug] the path optimizer dx_bounds_ assignee is finished! " << std::endl; 

		// calculate lateral acc bound
		lateral_acc_bound = std::tan(max_steering_wheel_angle) / wheel_base;
		for (uint32_t i = 0; i < path_size; i++)
		{
			double kappa = target_line.target_line_trace_points[i].curv;
			std::cout << "index: " << i << " kappa: " << kappa << std::endl; 
			path_optimizer.ddx_bounds_[i].first = -lateral_acc_bound - kappa;
			path_optimizer.ddx_bounds_[i].second = lateral_acc_bound - kappa;
		}
		std::cout << "[Debug] the path optimizer ddx_bounds_ assignee is finished! " << std::endl; 

		// dddx constraint
		double max_yaw_rate = max_steer_angle_rate / steer_ratio / 2.0;
		double jerk_bound = max_yaw_rate / wheel_base / std::fmax(init_state[1], 1.0);
		path_optimizer.dddx_bound_.first = -jerk_bound;
		path_optimizer.dddx_bound_.second = jerk_bound;
		std::cout << "[Debug] the path optimizer dddx_bound_ assignee is finished! " << std::endl; 

		// Optimize
		if (!path_optimizer.Optimize(max_iter))
		{
			std::cout << "[ERROR] path planner the optimize step is failed ! " << std::endl;
			return false;
		}

		std::cout << "[Debug] The path optimizer step is finished. " << std::endl;

		*x = path_optimizer.x_;
		*dx = path_optimizer.dx_;
		*ddx = path_optimizer.ddx_;

		return true;
	}

}  // namespace PlanningOptimizer
/******************************************************************************
 * Copyright 2020 The zhejiang lab Authors. All Rights Reserved.
 *****************************************************************************/
 /**
  * @file local_target_generator.cpp
  **/

#include "local_target_generator.h"

namespace localPlanner {
	LocalTargetGenerator::LocalTargetGenerator(const std::vector<double>& lon_samples, const std::shared_ptr<zjlmap::Map> Gmap)
	{
		cl_lon_samples_ = lon_samples;
		Gmap_ = Gmap;
		Clane_ = { 18, 0, -3 };  // random init current lane_id
		Is_new_goal_ = false;
		InitHistoryInfos();
		pre_true_speed_ = 0.0;
		is_body_z_init = false;
	}


	std::vector<TargetNode> LocalTargetGenerator::TargetsGenerating(Box2d& robot, std::vector<Box2d>& obstacles, const double& pre_max_kappa)
	{
		pre_max_curvature_ = pre_max_kappa;
		robot_ = robot;
		UpdateUGVSL(); // update ugv slz info  base on current road and next road
		robot.rlsl = robot_.rlsl;
		LOG_INFO("ugv slz: %d, %d, %d", robot_.rlsl.lane_id.road_id, robot_.rlsl.lane_id.section_idx, robot_.rlsl.lane_id.local_id);

		std::vector<TargetNode> targets;

		if (Is_new_goal_ && !ReRouting())
			return targets;
		else Is_new_goal_ = false;

		if (!TraverseRoutes() && !ReRouting()) return targets;

		if (Is_new_lane_)
		{
			UpdateHLMRLaneGraph();
			UpdateVirtualObstacles();
			Is_new_lane_ = false;
		}


		// update lane graph L M R HM

		UpdateLaneBaseInfo();  // update current lane curvature and speed limit

		UpdateFilteredObstacles(obstacles);

		obstacles = filtered_obstacles_;

		UpdateVirtualObstaclesPassInfo();

		UpdateLMRProb();  // update the probability of L M R

		UpdateLookaheadDist(); // update the length of local target 


		LOG_INFO("base probability L : %.2f, M : %.2f, R : %.2f", travel_prob_.l, travel_prob_.m, travel_prob_.r);
		LOG_INFO("guide probability L : %.2f, M : %.2f, R : %.2f", guide_prob_.l, guide_prob_.m, guide_prob_.r);
		LOG_INFO("lmr probability L : %.2f, M : %.2f, R : %.2f", lmrlane_prob_.l, lmrlane_prob_.m, lmrlane_prob_.r);
		LOG_INFO("LMR is active L : %.2f, M : %.2f, R : %.2f", hlmrlane_graph_.L.is_active, hlmrlane_graph_.M.is_active, hlmrlane_graph_.R.is_active);
		LOG_INFO("current lane: %d, %d, %d", Clane_.road_id, Clane_.section_idx, Clane_.local_id);
		if (Clane_index_ < (Gpath_.lane_id_vec.size() - 1))
		{
			zjlmap::LaneId next_lane = Gpath_.lane_id_vec[Clane_index_ + 1];
			LOG_INFO("next lane:  %d, %d, %d", next_lane.road_id, next_lane.section_idx, next_lane.local_id);
		}

		std::vector<TargetNode> RL_targets = GetLRTargets(); // L and R
		targets.insert(targets.end(), RL_targets.begin(), RL_targets.end());

		std::vector<TargetNode> HM_targets = GetHMTargets(); // H and R 
		targets.insert(targets.end(), HM_targets.begin(), HM_targets.end());

		return targets;
	}

	HLMRLane_graph LocalTargetGenerator::GetHLMRLaneGraph()
	{
		return hlmrlane_graph_;
	}

	void LocalTargetGenerator::UpdateUGVSL()
	{
		// std::cout<<"ugv pose xy : "<<std::setprecision(8)<<ugv_node.x<<","<<ugv_node.y<<std::endl;
		zjlmap::XYZ ugv_xy = { robot_.x, robot_.y , 0.0 };
		zjlmap::SLZ new_ugv_sl = zjlmap::EmptySLZ;

		if (Is_new_goal_)
		{
			zjlmap::ErrorCode ec = Gmap_->find_slz_global(ugv_xy, new_ugv_sl);
			if (ec == zjlmap::ErrorCode::kOK)
			{
				robot_.rlsl = std::move(new_ugv_sl);
				return;
			}
		}

		if (Gmap_->is_point_around_road(Clane_.road_id, 3.0, ugv_xy, new_ugv_sl))
		{
			robot_.rlsl = std::move(new_ugv_sl);
			return;
		}

		if (hlmrlane_graph_.HM.is_active && Gmap_->is_point_around_road(hlmrlane_graph_.HM.lane_info.id.road_id, 3.0, ugv_xy, new_ugv_sl))
		{
			robot_.rlsl = std::move(new_ugv_sl);
			return;
		}

		zjlmap::ErrorCode ec = Gmap_->find_slz_with_hint(ugv_xy, Clane_, new_ugv_sl);
		if (ec == zjlmap::ErrorCode::kOK)
		{
			robot_.rlsl = std::move(new_ugv_sl);
		}
	}

	void LocalTargetGenerator::UpdateGoal(const Pose& goal)
	{
		zjlmap::XYZ goal_xy = { goal.x, goal.y, 0.0 };
		zjlmap::ErrorCode ec = Gmap_->find_slz_global(goal_xy, goal_sl_);
		if (ec != zjlmap::ErrorCode::kOK)
		{
			LOG_ERRO("find goal slz failed.");
			return;
		}

		Is_new_goal_ = true;
	}

	bool LocalTargetGenerator::GetRouteByStartSLZ(const zjlmap::SLZ& start_slz, zjlmap::Route& Gpath)
	{
		zjlmap::Anchor start_a = Gmap_->create_anchor("start", start_slz);
		zjlmap::Anchor end_a = Gmap_->create_anchor("end", goal_sl_);
		zjlmap::AnchorArray way_point_list = zjlmap::EmptyAnchorArray;
		zjlmap::Route new_path = zjlmap::EmptyRoute;
		zjlmap::ErrorCode ec = Gmap_->plan_route(start_a, way_point_list, end_a, new_path);
		if (ec == zjlmap::ErrorCode::kOK && new_path.lane_id_vec.size() > 0)
		{
			Gpath = new_path;
			return true;
		}
		else
		{
			LOG_ERRO("Routing failed.");
			return false;
		}
	}



	bool LocalTargetGenerator::ReRouting()
	{
		LOG_INFO("Is rerouting...");
		zjlmap::Route new_path = zjlmap::EmptyRoute;
		if (GetRouteByStartSLZ(robot_.rlsl, new_path))
		{
			Gpath_ = new_path;
			Gpath_ = new_path;
			Clane_ = Gpath_.lane_id_vec[0];
			Clane_index_ = 0;
			Clane_info_ = Gmap_->query_lane_info(Clane_);
			Is_new_goal_ = false;
			Is_new_lane_ = true;
		}
		else
		{
			LOG_ERRO("ReRouting failed.");
			return false;
		}

		LOG_INFO("A new global path is accessed now.");
		for (size_t i = 0; i < Gpath_.lane_id_vec.size(); i++)
		{
			LOG_INFO("road id %d, %d, %d", Gpath_.lane_id_vec[i].road_id, Gpath_.lane_id_vec[i].section_idx, Gpath_.lane_id_vec[i].local_id);
		}

		LOG_ERRO("route length: %d", Gpath_.lane_id_vec.size());

		return true;
	}

	// if return false ,need to rerouting
	bool LocalTargetGenerator::TraverseRoutes()
	{
		LOG_INFO("current ugv road id  %d, %d, %d", robot_.rlsl.lane_id.road_id, robot_.rlsl.lane_id.section_idx, robot_.rlsl.lane_id.local_id);

		if (robot_.rlsl.lane_id == Clane_)
			return true;

		if (Clane_index_ < (Gpath_.lane_id_vec.size() - 1))
		{
			zjlmap::LaneId next_lane = Gpath_.lane_id_vec[Clane_index_ + 1];
			if (robot_.rlsl.lane_id == next_lane)
			{
				Clane_index_ += 1;
				Clane_ = next_lane;
				Clane_info_ = Gmap_->query_lane_info(Clane_);
				Is_new_lane_ = true;
				return true;
			}
		}

		return false;
	}

	TargetNode LocalTargetGenerator::ConstructTarget(const Lane_node& lane_node, const double& s, const double& len_s)
	{
		TargetNode target;
		target.sl.s = len_s;
		target.sl.l = 0.0;

		size_t s_index = std::min(static_cast<size_t>(fabs(s - lane_node.lane_info.begin) / lane_node.ds), lane_node.lane_points_info.size() - 1);
		target.pose.x = lane_node.lane_points_info[s_index].x;
		target.pose.y = lane_node.lane_points_info[s_index].y;
		target.pose.theta = lane_node.lane_points_info[s_index].hdg;
		// SLZ
		target.rlsl.l = lane_node.lane_points_info[s_index].l;
		target.rlsl.s = lane_node.lane_points_info[s_index].s;
		target.rlsl.z = lane_node.lane_points_info[s_index].z;
		target.rlsl.lane_id = lane_node.lane_points_info[s_index].lane_id;

		return target;
	}

	void LocalTargetGenerator::ConstructLRTarget(const double& s, const double& len_s, std::vector<TargetNode>& LR_targets)
	{
		if (hlmrlane_graph_.L.is_active && lmrlane_prob_.l > 0.0)
		{
			TargetNode left_target = ConstructTarget(hlmrlane_graph_.L, s, len_s);
			left_target.driving_prob = lmrlane_prob_.l;
			left_target.lane_type = LaneType::LEFT;
			LR_targets.push_back(std::move(left_target));
		}

		if (hlmrlane_graph_.R.is_active && lmrlane_prob_.r > 0.0)
		{
			TargetNode right_target = ConstructTarget(hlmrlane_graph_.R, s, len_s);
			right_target.driving_prob = lmrlane_prob_.r;
			right_target.lane_type = LaneType::RIGHT;
			LR_targets.push_back(std::move(right_target));
		}
	}

	std::vector<TargetNode> LocalTargetGenerator::GetLRTargets()
	{
		std::vector<TargetNode> LR_targets;

		if (Clane_index_ == (Gpath_.lane_id_vec.size() - 1))
		{
			double ds = std::min(fabs(goal_sl_.s - robot_.rlsl.s), cl_lon_samples_[0]);
			ConstructLRTarget(robot_.rlsl.s - SignofLane(Clane_)*ds, ds, LR_targets);
		}
		else
		{
			for (size_t i = 0; i < cl_lon_samples_.size(); i++)
			{
				double ds = cl_lon_samples_[i];
				if (fabs(Clane_info_.end - robot_.rlsl.s) > cl_lon_samples_[i])
				{
					ConstructLRTarget(robot_.rlsl.s - SignofLane(Clane_)*ds, cl_lon_samples_[i], LR_targets);
					continue;
				}

				ds = (cl_lon_samples_[i] - fabs(Clane_info_.end - robot_.rlsl.s));
				zjlmap::LaneInfo HMLane_info = hlmrlane_graph_.HM.lane_info;
				if (hlmrlane_graph_.HM.is_active)
				{
					ds = std::min(ds, fabs(HMLane_info.end - HMLane_info.begin));
					ConstructLRTarget(HMLane_info.begin - SignofLane(HMLane_info.id)*ds, cl_lon_samples_[i], LR_targets);
				}

				if (ds >= fabs(HMLane_info.end - HMLane_info.begin)) break;
			}
		}

		return LR_targets;
	}

	std::vector<TargetNode> LocalTargetGenerator::GetHMTargets()
	{
		std::vector<TargetNode> M_targets;
		TargetNode middle_target;
		if (lmrlane_prob_.m == 0.0)
		{
			return M_targets;
		}

		if (Clane_index_ == (Gpath_.lane_id_vec.size() - 1))
		{
			double len_s = std::min(fabs(goal_sl_.s - robot_.rlsl.s), lookahead_dist_);
			double s = robot_.rlsl.s - SignofLane(Clane_)*len_s;
			middle_target = ConstructTarget(hlmrlane_graph_.M, s, len_s);
		}
		else
		{
			if (hlmrlane_graph_.HM.is_active && fabs(Clane_info_.end - robot_.rlsl.s) <= lookahead_dist_)
			{
				Lane_node hm_node = hlmrlane_graph_.HM;
				double s_len = std::min(fabs(Clane_info_.end - robot_.rlsl.s) + fabs(hm_node.lane_info.end - hm_node.lane_info.begin),
					lookahead_dist_);
				double s = hm_node.lane_info.begin - SignofLane(hm_node.lane_info.id)*fabs(s_len - fabs(Clane_info_.end - robot_.rlsl.s));
				middle_target = ConstructTarget(hlmrlane_graph_.HM, s, s_len);
			}
			else
			{
				double len_s = std::min(fabs(Clane_info_.end - robot_.rlsl.s), lookahead_dist_);
				double s = robot_.rlsl.s - SignofLane(Clane_)*len_s;
				middle_target = ConstructTarget(hlmrlane_graph_.M, s, len_s);
			}
		}

		middle_target.driving_prob = lmrlane_prob_.m;
		middle_target.lane_type = LaneType::MIDDLE;

		M_targets.push_back(std::move(middle_target));
		return M_targets;
	}


	int LocalTargetGenerator::SignofLane(zjlmap::LaneId lane_id)
	{
		return (lane_id.local_id > 0) ? 1 : -1;
	}

	void LocalTargetGenerator::ResetHLMRLaneGraph()
	{
		hlmrlane_graph_.L.is_active = false;
		hlmrlane_graph_.M.is_active = false;
		hlmrlane_graph_.R.is_active = false;
		hlmrlane_graph_.HL.is_active = false;
		hlmrlane_graph_.HM.is_active = false;
		hlmrlane_graph_.HR.is_active = false;

		hlmrlane_graph_.L.ds = hlmrlane_graph_.M.ds = hlmrlane_graph_.R.ds = hlmrlane_graph_.HM.ds = 0.3;

		hlmrlane_graph_.L.lane_info = zjlmap::EmptyLaneInfo;
		hlmrlane_graph_.M.lane_info = zjlmap::EmptyLaneInfo;
		hlmrlane_graph_.R.lane_info = zjlmap::EmptyLaneInfo;
		hlmrlane_graph_.HM.lane_info = zjlmap::EmptyLaneInfo;
	}

	// clamp s in [begin_s, end_s] of lane 
	double LocalTargetGenerator::ClampSinLaneInterval(double s, zjlmap::LaneInfo lane_info)
	{
		double new_s = s;
		if (SignofLane(lane_info.id)*s < SignofLane(lane_info.id)*lane_info.end)
		{
			new_s = lane_info.end;
		}

		if (SignofLane(lane_info.id)*s > SignofLane(lane_info.id)*lane_info.begin)
		{
			new_s = lane_info.begin;
		}

		return new_s;
	}

	bool LocalTargetGenerator::ConstructLaneNode(localPlanner::Lane_node& lane_node)
	{
		// get lane center line SL-L
		std::vector<zjlmap::TracePoint> lane_points_info;
		zjlmap::LaneInfo lane_info = lane_node.lane_info;
		zjlmap::ErrorCode ec = Gmap_->calc_lane_center_line_curv(lane_info.id, lane_info.begin, lane_info.end, lane_node.ds, lane_points_info);

		LOG_INFO("construct lane info for lane id: %d, %d, %d", lane_node.lane_info.id.road_id, lane_node.lane_info.id.section_idx, lane_node.lane_info.id.local_id);
		if (ec == zjlmap::ErrorCode::kOK)
		{
			// for (size_t i = 0; i < lane_points_info.size(); i++)
			// {
			//     std::cout<<i<<": "<<std::setprecision(10)<<lane_points_info[i].x<<", "<<lane_points_info[i].y<<std::endl;
			// }
			lane_node.lane_points_info = std::move(lane_points_info);
			return true;
		}
		else
		{
			LOG_ERRO("calc lane center line failed.");
			return false;
		}
	}

	void LocalTargetGenerator::UpdateNextLaneType()
	{
		// HM
		next_lanetype_ = localPlanner::LaneType::MIDDLE;
		if (Clane_index_ < (Gpath_.lane_id_vec.size() - 1))
		{
			zjlmap::LaneId next_lane = Gpath_.lane_id_vec[Clane_index_ + 1];
			if (next_lane.road_id == Clane_.road_id && next_lane.section_idx == Clane_.section_idx)
			{
				if (SignofLane(Clane_info_.id)*next_lane.local_id < SignofLane(Clane_info_.id)*Clane_.local_id) // L  
					next_lanetype_ = localPlanner::LaneType::LEFT;

				if (SignofLane(Clane_info_.id)*next_lane.local_id > SignofLane(Clane_info_.id)*Clane_.local_id)  // R
					next_lanetype_ = localPlanner::LaneType::RIGHT;
			}
		}
	}
	// udpate only current lane id changed
	void LocalTargetGenerator::UpdateHLMRLaneGraph()
	{
		UpdateNextLaneType();
		ResetHLMRLaneGraph();
		hlmrlane_graph_.L.is_active = false;
		hlmrlane_graph_.M.is_active = false;
		hlmrlane_graph_.R.is_active = false;
		hlmrlane_graph_.HM.is_active = false;

		// M
		hlmrlane_graph_.M.is_active = true;
		hlmrlane_graph_.M.lane_info = Clane_info_;
		if (!ConstructLaneNode(hlmrlane_graph_.M))
			LOG_ERRO("construct middle lane node failed.");

		// get current lane linkage information
		zjlmap::LaneLinkage lane_linkage;
		zjlmap::ErrorCode ec = Gmap_->query_lane_linkage(Clane_info_.id, lane_linkage);

		if (ec != zjlmap::ErrorCode::kOK)
		{
			LOG_ERRO("query lane linkage failed!");
			return;
		}

		// L
		if (fabs(lane_linkage.left_neighbor.local_id) < 99)
		{
			hlmrlane_graph_.L.is_active = true;
			hlmrlane_graph_.L.lane_info = Gmap_->query_lane_info(lane_linkage.left_neighbor);
			if (!ConstructLaneNode(hlmrlane_graph_.L))
				LOG_ERRO("construct left lane node failed.");

			//     std::ofstream Lftxtx("/apollo/modules/zj_planning/data/L_lane_center_x.txt", std::ios::app);
			//     std::ofstream Lftxty("/apollo/modules/zj_planning/data/L_lane_center_y.txt", std::ios::app);

			//     for (size_t i = 0; i < hlmrlane_graph_.L.lane_points_info.size(); i++)
			//     {
			//         Lftxtx<<std::setprecision(10)<<hlmrlane_graph_.L.lane_points_info[i].x<<std::endl; 
			//         Lftxty<<std::setprecision(10)<<hlmrlane_graph_.L.lane_points_info[i].y<<std::endl;
			//     }                
		}

		// R
		if (fabs(lane_linkage.right_neighbor.local_id) < 99)
		{
			hlmrlane_graph_.R.is_active = true;
			hlmrlane_graph_.R.lane_info = Gmap_->query_lane_info(lane_linkage.right_neighbor);
			if (!ConstructLaneNode(hlmrlane_graph_.R))
				LOG_ERRO("construct right lane node failed.");
			// std::ofstream Rftxtx("/apollo/modules/zj_planning/data/R_lane_center_x.txt", std::ios::app);
			// std::ofstream Rftxty("/apollo/modules/zj_planning/data/R_lane_center_y.txt", std::ios::app);

			// for (size_t i = 0; i < hlmrlane_graph_.R.lane_points_info.size(); i++)
			// {
			//     Rftxtx<<std::setprecision(10)<<hlmrlane_graph_.R.lane_points_info[i].x<<std::endl; 
			//     Rftxty<<std::setprecision(10)<<hlmrlane_graph_.R.lane_points_info[i].y<<std::endl;
			// }      
		}

		// HM
		if (Clane_index_ < (Gpath_.lane_id_vec.size() - 1) && next_lanetype_ == localPlanner::LaneType::MIDDLE)
		{
			hlmrlane_graph_.HM.is_active = true;
			hlmrlane_graph_.HM.lane_info = Gmap_->query_lane_info(Gpath_.lane_id_vec[Clane_index_ + 1]);
		}
		else
		{
			if (lane_linkage.successor_lanes.size() == 1)
			{
				hlmrlane_graph_.HM.is_active = true;
				hlmrlane_graph_.HM.lane_info = Gmap_->query_lane_info(lane_linkage.successor_lanes[0]);
			}

			if (lane_linkage.successor_lanes.size() > 1)
			{
				double min_length = FLT_MAX;
				for (size_t i = 0; i < lane_linkage.successor_lanes.size(); i++)
				{
					zjlmap::LaneInfo temp_lane_info = Gmap_->query_lane_info(lane_linkage.successor_lanes[i]);
					zjlmap::SLZArray slz_arr = zjlmap::EmptySLZArray;
					zjlmap::Route temp_path = zjlmap::EmptyRoute;
					ec = Gmap_->calc_lane_center_line(temp_lane_info.id, temp_lane_info.begin - 0.1, temp_lane_info.begin + 0.1, 0.01, slz_arr);
					if (ec != zjlmap::ErrorCode::kOK)
					{
						LOG_ERRO("calc lane center line failed!");
						return;
					}
					if (GetRouteByStartSLZ(slz_arr[i], temp_path))
					{
						hlmrlane_graph_.HM.is_active = true;
						if (temp_path.length < min_length)
						{
							min_length = temp_path.length;
							hlmrlane_graph_.HM.lane_info = temp_lane_info;
						}
					}
				}
			}
		}

		if (hlmrlane_graph_.HM.is_active)
		{
			if (!ConstructLaneNode(hlmrlane_graph_.HM))
				LOG_ERRO("construct ahead middle lane node failed.");
		}
		LOG_ERRO("update hlmrlane graph success.");
	}

	// udpate at every decision loop
	void LocalTargetGenerator::UpdateFilteredObstacles(const std::vector<Box2d>& obstacles)
	{
		filtered_obstacles_.clear();

		for (size_t i = 0; i < obstacles.size(); i++)
		{
			Box2d obs = obstacles[i];

			zjlmap::XYZ obs_xy = { obs.x, obs.y, 0.0 };

			double pre_t = 1.5; // default: predictive duration is 3.0 seconds
			if (obs.roleType == RoleType::PEDESTRIAN) pre_t = 3.0; // pedestrian: predictive duration is 3.0 seconds

			if (Gmap_->is_point_around_road(Clane_.road_id, pre_t*obs.speed, obs_xy, obs.rlsl))
			{
				int s_index = int(fabs(obs.rlsl.s - hlmrlane_graph_.M.lane_info.begin) / hlmrlane_graph_.M.ds);
				double lane_l = hlmrlane_graph_.M.lane_points_info[s_index].l;
				double lane_theta = hlmrlane_graph_.M.lane_points_info[s_index].hdg;
				double pl = Project2LaneLAxis(lane_theta, obs);
				double correct_width_l = 3.0;
				double correct_speed_l = 0.0;

				if (SignofLane(hlmrlane_graph_.M.lane_info.id)*lane_l > SignofLane(hlmrlane_graph_.M.lane_info.id)*obs.rlsl.l)
				{
					// obstacle is on the left of current lane center
					correct_width_l += (3.0*static_cast<float>(hlmrlane_graph_.L.is_active));
					correct_speed_l = std::min(0.0, pre_t*obs.speed*std::cos(obs.theta - lane_theta + M_PI / 2.0));
				}
				else
				{
					// obstacle is on the right of current lane center
					correct_width_l += (3.0*static_cast<float>(hlmrlane_graph_.R.is_active));
					correct_speed_l = std::min(0.0, pre_t*obs.speed*std::cos(obs.theta - lane_theta - M_PI / 2.0));
				}


				if (fabs(lane_l - obs.rlsl.l) < (pl / 2.0 + correct_width_l + correct_speed_l))
				{
					filtered_obstacles_.push_back(obs);
					continue;
				}

			}

			if (hlmrlane_graph_.HM.is_active && Gmap_->is_point_around_road(hlmrlane_graph_.HM.lane_info.id.road_id, pre_t*obs.speed, obs_xy, obs.rlsl))
			{
				int s_index = int(fabs(obs.rlsl.s - hlmrlane_graph_.HM.lane_info.begin) / hlmrlane_graph_.HM.ds);
				double lane_l = hlmrlane_graph_.HM.lane_points_info[s_index].l;
				double lane_theta = hlmrlane_graph_.HM.lane_points_info[s_index].hdg;
				double pl = Project2LaneLAxis(lane_theta, obs);
				double correct_width_l = 3.0;
				double correct_speed_l = 0.0;

				if (SignofLane(hlmrlane_graph_.HM.lane_info.id)*lane_l > SignofLane(hlmrlane_graph_.HM.lane_info.id)*obs.rlsl.l)
				{
					// obstacle is on the left of current lane center
					correct_speed_l = std::min(0.0, pre_t*obs.speed*std::cos(obs.theta - lane_theta + M_PI / 2.0));
				}
				else
				{
					// obstacle is on the right of current lane center
					correct_speed_l = std::min(0.0, pre_t*obs.speed*std::cos(obs.theta - lane_theta - M_PI / 2.0));
				}

				if (fabs(lane_l - obs.rlsl.l) < (pl / 2.0 + correct_width_l + correct_speed_l))
				{
					filtered_obstacles_.push_back(obs);
				}
			}
		}
		filtered_obstacles_.insert(filtered_obstacles_.end(), static_obstacles_.begin(), static_obstacles_.end());

		// for (const auto& obs:virtual_obstacles_)
		//     filtered_obstacles_.push_back(obs);
	}
	// update at every decision loop
	void LocalTargetGenerator::UpdateTravelProb()
	{
		travel_prob_.m = travel_prob_.r = travel_prob_.l = 0.0;

		// by traffic rules
		double test_s = robot_.rlsl.s - 10.0*SignofLane(robot_.rlsl.lane_id);
		zjlmap::LaneChangeType lct = Gmap_->query_lane_change_type_at(Clane_, test_s);

		switch (lct)
		{
		case zjlmap::LaneChangeType::kLeftChange:
			travel_prob_.m = travel_prob_.l = 1.0; break;
		case zjlmap::LaneChangeType::kRightChange:
			travel_prob_.m = travel_prob_.r = 1.0; break;
		case zjlmap::LaneChangeType::kBothChange:
			travel_prob_.m = travel_prob_.l = travel_prob_.r = 1.0; break;
		default:
			travel_prob_.m = 1.0; break;
		}

		double dtheta = 0.0; // the angle between obstacle heading and lane heading
		double ds = 0.0;

		// by static obstacles
		for (size_t i = 0; i < filtered_obstacles_.size(); i++)
		{
			Box2d obs = filtered_obstacles_[i];
			// 如果障碍物为动态或者障碍物类型为行人时 忽略
			if (!obs.is_static || (obs.roleType == localPlanner::RoleType::PEDESTRIAN)) continue;

			// M 只需要对满交通规则的车道进行后续判断
			if (travel_prob_.m == 1.0 && IsobstacleBlockLane(hlmrlane_graph_.M, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.M.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);
				if (obs2ugv_dist > 0.0 && obs2ugv_dist < 30.0) travel_prob_.m = 0.0;
			}

			if (travel_prob_.r == 1.0 && IsobstacleBlockLane(hlmrlane_graph_.R, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.R.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);
				if (obs2ugv_dist > 0.0 && obs2ugv_dist < 30.0) travel_prob_.r = 0.0;
			}

			if (travel_prob_.l == 1.0 && IsobstacleBlockLane(hlmrlane_graph_.L, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.L.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);
				if (obs2ugv_dist > 0.0 && obs2ugv_dist < 30.0) travel_prob_.l = 0.0;
			}

			if (travel_prob_.m == 1.0 && IsobstacleBlockLane(hlmrlane_graph_.HM, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.HM.lane_info.id)*(obs.rlsl.s - hlmrlane_graph_.HM.lane_info.begin);
				if (obs2ugv_dist > 0.0 && obs2ugv_dist < (30.0 - fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s))) travel_prob_.m = 0.0;
			}
		}

		// get the break out probability from current block
		if (travel_prob_.m == 0.0 && travel_prob_.l == 0.0 && travel_prob_.r == 0.0)
		{
			double test_s = robot_.rlsl.s - 15.0*SignofLane(Clane_info_.id);
			test_s = ClampSinLaneInterval(test_s, Clane_info_);
			if (fabs(test_s - robot_.rlsl.s) > 10.0)
			{
				zjlmap::LaneChangeType lct_left = Gmap_->query_lane_change_type_at(hlmrlane_graph_.L.lane_info.id, test_s);
				if (lct_left == zjlmap::LaneChangeType::kLeftChange || lct_left == zjlmap::LaneChangeType::kBothChange)
				{
					travel_prob_.l = 0.2;
				}

				zjlmap::LaneChangeType lct_right = Gmap_->query_lane_change_type_at(hlmrlane_graph_.R.lane_info.id, test_s);
				if (lct_right == zjlmap::LaneChangeType::kRightChange || lct_right == zjlmap::LaneChangeType::kBothChange)
				{
					travel_prob_.r = 0.2;
				}
			}
		}
		// by dynamic obstacles
		for (size_t i = 0; i < filtered_obstacles_.size(); i++)
		{
			Box2d obs = filtered_obstacles_[i];
			// 如果为动态障碍物时
			if (obs.is_static) continue;

			if (travel_prob_.m > 0.0 && IsobstacleBlockLane(hlmrlane_graph_.M, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.M.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);
				if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
				{
					travel_prob_.m = 0.0;
				}
			}

			if (travel_prob_.r > 0.0 && IsobstacleBlockLane(hlmrlane_graph_.R, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.R.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);

				if (std::cos(dtheta) > 0.8 && obs2ugv_dist > -5.0 && obs2ugv_dist < 10.0)
				{
					travel_prob_.r = 0.0;
					break;
				}

				if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
				{
					travel_prob_.r = 0.0;
					break;
				}

				if (std::cos(dtheta + M_PI / 2.0) > 0.5 || std::cos(dtheta - M_PI / 2.0) > 0.5)
				{
					if (obs2ugv_dist > 1.0 && obs2ugv_dist < 15.0)
					{
						travel_prob_.r = 0.0;
						break;
					}
				}

				// if ((obs2ugv_dist + ds) > 1.0 && obs2ugv_dist < 10.0)
				// {
				//     travel_prob_.r = 0.0;
				// }
				// else if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
				// {
				//     travel_prob_.r = 0.0;
				// }
			}

			if (travel_prob_.l > 0.0 && IsobstacleBlockLane(hlmrlane_graph_.L, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.L.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);

				if (std::cos(dtheta) > 0.8 && obs2ugv_dist > -5.0 && obs2ugv_dist < 10.0)
				{
					travel_prob_.l = 0.0;
					break;
				}

				if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
				{
					travel_prob_.l = 0.0;
					break;
				}

				if (std::cos(dtheta + M_PI / 2.0) > 0.5 || std::cos(dtheta - M_PI / 2.0) > 0.5)
				{
					if (obs2ugv_dist > 1.0 && obs2ugv_dist < 15.0)
					{
						travel_prob_.l = 0.0;
						break;
					}
				}
				// if ((obs2ugv_dist + ds) > 1.0 && obs2ugv_dist < 10.0)
				// {
				//     travel_prob_.l = 0.0;
				// }
				// else if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
				// {
				//     travel_prob_.l = 0.0;
				// }
			}

			// if (travel_prob_.m == 1.0 && IsobstacleBlockLane(hlmrlane_graph_.HM, obs, dtheta))
			// {
			//     double obs2ugv_dist = -SignofLane(hlmrlane_graph_.HM.lane_info.id)*(obs.rlsl.s - hlmrlane_graph_.HM.lane_info.begin);
			//     if (std::cos(dtheta + M_PI) > 0.8 && obs2ugv_dist > 0.0)
			//     {
			//         travel_prob_.m = 0.0;
			//     }                         
			// }
		}
	}

	void LocalTargetGenerator::UpdateGuideProb()
	{

		if (Clane_index_ < (Gpath_.lane_id_vec.size() - 1)) // L and R
		{
			zjlmap::LaneId next_lane = Gpath_.lane_id_vec[Clane_index_ + 1];
			if ((next_lane.road_id == Clane_.road_id) && (next_lane.section_idx == Clane_.section_idx))
			{
				double ds = fabs(robot_.rlsl.s - Clane_info_.end);
				double s_len = fabs(Clane_info_.end - Clane_info_.begin);
				double si = 0.0;
				double se = s_len - 0.0;
				//double si = 2.0;
				//double se = s_len - 10.0;
				double change_prob = 0.3*(ds - si)*(ds - se) / ((s_len / 2.0 - si)*(s_len / 2.0 - se)) + 0.5;

				if (IsGoalInCurrentRoad())
				{
					ds = fabs(robot_.rlsl.s - goal_sl_.s);
					if (ds < 15.0)
						change_prob = 1.0;
				}


				if (SignofLane(Clane_info_.id)*next_lane.local_id < SignofLane(Clane_info_.id)*Clane_.local_id) // L
				{
					LOG_INFO("next lane is on the Left.");
					guide_prob_.l = change_prob;
					guide_prob_.m = (1.0 - change_prob);
					guide_prob_.r = std::min(0.1, (1.0 - change_prob));
					return;
				}

				if (SignofLane(Clane_info_.id)*next_lane.local_id > SignofLane(Clane_info_.id)*Clane_.local_id) // R
				{
					LOG_INFO("next lane is on the Right");
					guide_prob_.l = std::min(0.1, (1.0 - change_prob));
					guide_prob_.m = (1.0 - change_prob);
					guide_prob_.r = change_prob;
					return;
				}

				LOG_ERRO("the next lane is wrong....");
			}
		}

		// M and H
		// bool Is_goal_in_current_lane = false;
		double ds = fabs(Clane_info_.end - robot_.rlsl.s);
		LOG_INFO("next lane is on the ahead");
		if (Clane_index_ == (Gpath_.lane_id_vec.size() - 1) && robot_.rlsl.lane_id == goal_sl_.lane_id)
		{
			ds = fabs(goal_sl_.s - robot_.rlsl.s);
			// Is_goal_in_current_lane = true;
		}

		double keep_prob = std::min(1.0 / std::exp(0.01*(ds + 50.0)) + 0.5, 1.0);
		guide_prob_.l = (1.0 - keep_prob) / 2.0;
		guide_prob_.m = keep_prob;
		guide_prob_.r = (1.0 - keep_prob) / 2.0;
	}

	void LocalTargetGenerator::UpdateLMRProb()
	{
		UpdateTravelProb(); // udpate base probability
		UpdateGuideProb(); // update guide probability
		// TODO LIST other probability ... 

		lmrlane_prob_.l = travel_prob_.l * guide_prob_.l;
		lmrlane_prob_.m = travel_prob_.m * guide_prob_.m;
		lmrlane_prob_.r = travel_prob_.r * guide_prob_.r;

		// form in norm probability
		double sum_prob = lmrlane_prob_.l + lmrlane_prob_.m + lmrlane_prob_.r;
		if (sum_prob == 0.0)
		{
			lmrlane_prob_.l = 0.0;
			lmrlane_prob_.m = 0.0;
			lmrlane_prob_.r = 0.0;
		}
		else
		{
			lmrlane_prob_.l = lmrlane_prob_.l / sum_prob;
			lmrlane_prob_.m = lmrlane_prob_.m / sum_prob;
			lmrlane_prob_.r = lmrlane_prob_.r / sum_prob;
		}

		if (lmrlane_prob_.l == 1.0) lmrlane_prob_.l = 0.995;

		if (lmrlane_prob_.m == 1.0) lmrlane_prob_.m = 0.995;

		if (lmrlane_prob_.r == 1.0) lmrlane_prob_.r = 0.995;
	}

	bool LocalTargetGenerator::IsGoalInCurrentRoad()
	{
		// on the same road and the lane one the same direction
		if (goal_sl_.lane_id.road_id == robot_.rlsl.lane_id.road_id
			&& goal_sl_.lane_id.section_idx == robot_.rlsl.lane_id.section_idx
			&& goal_sl_.lane_id.local_id*robot_.rlsl.lane_id.local_id > 0.0)
		{
			return true;
		}
		else
		{
			return false;
		}

	}
	// update at every decision loop
	void LocalTargetGenerator::UpdateLaneBaseInfo()
	{
		double speed_limit = 0.0;
		zjlmap::ErrorCode ec = Gmap_->query_lane_speed_at(Clane_, robot_.rlsl.s, speed_limit);
		if (ec == zjlmap::ErrorCode::kOK)
		{
			lane_max_speed_ = speed_limit;
		}
		else
		{
			LOG_ERRO("get speed limit at current lane failed!!!");
			return;
		}

		lane_curvature_abs_ = 0.0;


		size_t s_index = static_cast<size_t>(fabs(robot_.rlsl.s - hlmrlane_graph_.M.lane_info.begin) / hlmrlane_graph_.M.ds);
		size_t s_num = static_cast<size_t>(std::min(25.0, fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s)) / hlmrlane_graph_.M.ds);
		for (size_t i = s_index; i < std::min(s_index + s_num, hlmrlane_graph_.M.lane_points_info.size()); i++)
			lane_curvature_abs_ = std::max(fabs(hlmrlane_graph_.M.lane_points_info[i].curv), lane_curvature_abs_);

		s_num = static_cast<size_t>(std::min(10.0, fabs(hlmrlane_graph_.M.lane_info.begin - robot_.rlsl.s)) / hlmrlane_graph_.M.ds);
		for (size_t i = s_index; i > std::max(s_index - s_num, static_cast<size_t>(0)); i--)
			lane_curvature_abs_ = std::max(fabs(hlmrlane_graph_.M.lane_points_info[i].curv), lane_curvature_abs_);

		if (hlmrlane_graph_.HM.is_active && fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s) < 25.0)
		{
			s_num = static_cast<size_t>(std::min(25.0 - fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s),
				fabs(hlmrlane_graph_.HM.lane_info.end - hlmrlane_graph_.HM.lane_info.begin)) / hlmrlane_graph_.M.ds);

			for (size_t i = 0; i < std::min(s_num, hlmrlane_graph_.HM.lane_points_info.size()); i++)
				lane_curvature_abs_ = std::max(fabs(hlmrlane_graph_.HM.lane_points_info[i].curv), lane_curvature_abs_);
		}
	}

	void LocalTargetGenerator::UpdateLookaheadDist()
	{
		if (lmrlane_prob_.m == 0.0) return;

		LOG_INFO("pre max kappa: %.2f, lane curvature abs: %.2f", pre_max_curvature_, lane_curvature_abs_);

		lookahead_dist_ = std::max(7.0, std::min(std::exp(-18.0*(std::max(lane_curvature_abs_, pre_max_curvature_) - 0.2)) + 0.6, 30.0));

		// lookahead_dist_ = std::max(7.0, std::min(1.4/(std::max(lane_curvature_abs_, pre_max_curvature_)+0.0001), 30.0));

		LOG_INFO("lookahead dist based on curvature: %.2f", lookahead_dist_);
		// by real dynamic obstacles
		for (size_t i = 0; i < filtered_obstacles_.size(); i++)
		{
			Box2d obs = filtered_obstacles_[i];
			if (obs.is_static) continue;

			double dx = robot_.x - obs.x;
			double dy = robot_.y - obs.y;
			double angle_ugv_obs = std::atan2(dy, dx);
			if (std::cos(angle_ugv_obs - robot_.theta) > 0.85) continue;


			// double dist = std::sqrt(dx*dx+dy*dy);

			// if (dist > 25.0) continue;            

			double correct_dist = 4.0;
			if (obs.roleType == localPlanner::RoleType::PEDESTRIAN)
			{
				// std::cout<<"this is pedestran...,dist is:";
				correct_dist = 12.0;
			}

			// double dx = obs.x - robot_.x;
			// double dy = obs.y - robot_.y;
			// double dist = std::sqrt(dx*dx+dy*dy);

			// if (dist > 25.0) continue;

			double dtheta = 0.0;
			double ds = 0.0;

			if (IsobstacleBlockLane(hlmrlane_graph_.M, obs, dtheta, ds))
			{
				double correct_obs_s = obs.rlsl.s - SignofLane(hlmrlane_graph_.M.lane_info.id)*ds;
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.M.lane_info.id)*(correct_obs_s - robot_.rlsl.s);
				double correct_obs2ugv_dist = std::max(0.0, obs2ugv_dist - (std::sqrt(obs.length*obs.length + obs.width*obs.width) / 2.0 + correct_dist));
				if (obs2ugv_dist > 0.0) lookahead_dist_ = std::min(correct_obs2ugv_dist, lookahead_dist_);
				continue;
			}

			if (hlmrlane_graph_.HM.is_active && IsobstacleBlockLane(hlmrlane_graph_.HM, obs, dtheta, ds))
			{
				double correct_obs_s = obs.rlsl.s - SignofLane(hlmrlane_graph_.HM.lane_info.id)*ds;
				double obs2ugv_dist = fabs(correct_obs_s - hlmrlane_graph_.HM.lane_info.begin)
					+ fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s);
				double correct_obs2ugv_dist = std::max(0.0, obs2ugv_dist - (std::sqrt(obs.length*obs.length + obs.width*obs.width) / 2.0 + correct_dist));
				if (obs2ugv_dist > 0.0) lookahead_dist_ = std::min(correct_obs2ugv_dist, lookahead_dist_);
			}
		}

		// by virtual obstacles
		for (size_t i = 0; i < virtual_obstacles_.size(); i++)
		{
			Box2d obs = virtual_obstacles_[i];
			size_t obs_indx = virtual_obs_indx_[i];
			double desired_v = history_infos_[obs_indx].speed_pass + (history_infos_[obs_indx].speed_limit - history_infos_[obs_indx].speed_pass) / 3.0;
			double desired_l = (desired_v + 2.0)*(desired_v + 2.0) / 5.0;
			double dtheta = 0.0;
			double ds = 0.0;

			if (IsobstacleBlockLane(hlmrlane_graph_.M, obs, dtheta, ds))
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.M.lane_info.id)*(obs.rlsl.s - robot_.rlsl.s);
				if (obs2ugv_dist > -3.0 && obs2ugv_dist < 25.0)
					lookahead_dist_ = std::min(desired_l, lookahead_dist_);
				continue;
			}

			if (hlmrlane_graph_.HM.is_active && IsobstacleBlockLane(hlmrlane_graph_.HM, obs, dtheta, ds))
			{
				double obs2ugv_dist = fabs(obs.rlsl.s - hlmrlane_graph_.HM.lane_info.begin)
					+ fabs(hlmrlane_graph_.M.lane_info.end - robot_.rlsl.s);
				if (obs2ugv_dist > -3.0 && obs2ugv_dist < 25.0)
					lookahead_dist_ = std::min(desired_l, lookahead_dist_);
			}
		}

		if (IsGoalInCurrentRoad())
		{
			lookahead_dist_ = std::min(std::max(0.0, -SignofLane(Clane_)*(goal_sl_.s - robot_.rlsl.s)
				- 15.0*fabs(goal_sl_.lane_id.local_id - robot_.rlsl.lane_id.local_id)), lookahead_dist_);
			// if the goal is one the same road but not same lane,
			//  use local id diff: 10.0*fabs(goal local id - ugv local id) to correct the behaviour.
			LOG_INFO("look head dist based on goal: %.2f", lookahead_dist_);
		}

		LOG_INFO("final lookahead dist: %.2f", lookahead_dist_);
	}


	bool LocalTargetGenerator::IsobstacleBlockLane(const Lane_node& lane_node, const Box2d& obs, double& dtheta, double& ds)
	{
		if (obs.rlsl.lane_id.road_id != lane_node.lane_info.id.road_id)
			return false;

		int s_index = int(fabs(obs.rlsl.s - lane_node.lane_info.begin) / lane_node.ds);
		std::cout << "[Debug] s_index: " << s_index << std::endl;  
		std::cout << "[Debug] lane_node.lane_points_info.size: " << lane_node.lane_points_info.size() << std::endl; 
		double lane_l = lane_node.lane_points_info[s_index].l;
		double lane_theta = hlmrlane_graph_.M.lane_points_info[s_index].hdg;
		double pl = Project2LaneLAxis(lane_theta, obs);
		double correct_width_l = (robot_.width + pl) / 2.0;
		double correct_speed_l = 0.0;  // if dynamic obstacle 
		dtheta = obs.theta - lane_theta;
		ds = 0.0;

		if (!obs.is_static)
		{
			double pre_t = 1.5;
			if (obs.roleType == RoleType::PEDESTRIAN) pre_t = 3.0;
			double sin_dtheta = std::sin(dtheta);
			if ((SignofLane(lane_node.lane_info.id)*lane_l > SignofLane(lane_node.lane_info.id)*obs.rlsl.l && sin_dtheta < 0.0) ||
				(SignofLane(lane_node.lane_info.id)*lane_l < SignofLane(lane_node.lane_info.id)*obs.rlsl.l && sin_dtheta > 0.0))
			{
				// obstacle is on the left of current lane center
				correct_speed_l = std::min(pre_t*obs.speed*fabs(sin_dtheta), fabs(lane_l - obs.rlsl.l));
				ds = fabs(lane_l - obs.rlsl.l) * std::cos(dtheta) / fabs(sin_dtheta);
			}

		}
		double correct_dl = 0.0;
		if (obs.roleType == RoleType::PEDESTRIAN) correct_dl = 0.5;

		if ((fabs(lane_l - obs.rlsl.l) + correct_dl) < (correct_width_l + correct_speed_l))
		{
			return true;
		}

		return false;
	}


	double LocalTargetGenerator::Project2LaneLAxis(const double& lane_theta, const Box2d& obs)
	{
		return obs.length*fabs(std::sin(obs.theta - lane_theta)) + obs.width*fabs(std::cos(obs.theta - lane_theta));
	}


	// History info funcs

	void LocalTargetGenerator::SaveHistoryInfos()
	{
		std::ofstream outfile("./data/history_infos.txt"); // TODO LIST
		for (size_t i = 0; i < history_infos_.size(); i++)
		{
			outfile << history_infos_[i].event_slz.lane_id.road_id << " " <<
				history_infos_[i].event_slz.lane_id.section_idx << " " <<
				history_infos_[i].event_slz.lane_id.local_id << " " <<
				history_infos_[i].event_slz.s << " " <<
				history_infos_[i].event_slz.l << " " <<
				history_infos_[i].prob << " " <<
				history_infos_[i].speed_pass << " " <<
				history_infos_[i].speed_limit << " " <<
				history_infos_[i].event_type << std::endl;
		}
		outfile.close();
	}

	void LocalTargetGenerator::InitHistoryInfos()
	{
		history_infos_.clear();

		std::ifstream infile("./data/history_infos.txt");
		HistoryInfo his_info;
		while (infile >> his_info.event_slz.lane_id.road_id
			>> his_info.event_slz.lane_id.section_idx
			>> his_info.event_slz.lane_id.local_id
			>> his_info.event_slz.s
			>> his_info.event_slz.l
			>> his_info.prob
			>> his_info.speed_pass
			>> his_info.speed_limit
			>> his_info.event_type)
		{
			his_info.is_active = false;
			his_info.is_meet = false;
			his_info.is_pass = false;
			history_infos_.push_back(his_info);
			LOG_INFO("event info: %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %d",
				his_info.event_slz.lane_id.road_id,
				his_info.event_slz.lane_id.section_idx,
				his_info.event_slz.lane_id.local_id,
				his_info.event_slz.s,
				his_info.event_slz.l,
				his_info.prob,
				his_info.speed_pass,
				his_info.speed_limit,
				his_info.event_type);
		}

		infile.close();
	}

	void LocalTargetGenerator::AbnormStatusDetection(const double& pre_desired_speed)
	{

		if (!is_body_z_init)
		{
			pre_body_z = robot_.z;
			is_body_z_init = true;
		}

		// by speed stick 
		LOG_INFO("pre_desired_speed: %.2f, pre_true_speed: %.2f, true_speed: %.2f", pre_desired_speed, pre_true_speed_, robot_.speed);

		if (pre_desired_speed > pre_true_speed_)
		{
			if ((pre_true_speed_ - robot_.speed) > 1.0)
			{
				AddOneHistoryInfo(1);
				pre_true_speed_ = robot_.speed;
				pre_body_z = robot_.z;
				return;
			}
		}

		// by speed slip
		if ((pre_true_speed_ - pre_desired_speed) > 3.0)
		{
			if ((robot_.speed - pre_true_speed_) > 1.0)
			{
				AddOneHistoryInfo(2);
				pre_true_speed_ = robot_.speed;
				pre_body_z = robot_.z;
				return;
			}
		}

		// by body z bump
		// if (fabs(pre_body_z - robot_.z) > 0.2)
		// {
		//     AddOneHistoryInfo(3);
		//     pre_true_speed_ = robot_.speed;
		//     pre_body_z = robot_.z; 
		//     return;          
		// }

		pre_true_speed_ = robot_.speed;
		pre_body_z = robot_.z;


	}

	void LocalTargetGenerator::AddOneHistoryInfo(int event_type)
	{
		for (size_t i = 0; i < history_infos_.size(); i++)
		{
			if (robot_.rlsl.lane_id == history_infos_[i].event_slz.lane_id)
			{
				if (fabs(robot_.rlsl.s - history_infos_[i].event_slz.s) < 15.0)
				{
					if (history_infos_[i].is_active)
					{
						history_infos_[i].event_slz = robot_.rlsl;
						history_infos_[i].event_type = event_type;
						history_infos_[i].is_meet = false;
						history_infos_[i].is_active = false;
						history_infos_[i].is_pass = false;
						history_infos_[i].speed_limit = pre_true_speed_;
						history_infos_[i].speed_pass = 0.0;
						history_infos_[i].prob = 1.0;
						SaveHistoryInfos();
					}
					return;
				}
			}
		}

		HistoryInfo now_info;
		now_info.event_slz = robot_.rlsl;
		now_info.prob = 1.0;
		now_info.speed_limit = pre_true_speed_;
		now_info.speed_pass = 0.0;
		now_info.event_type = event_type;
		now_info.is_active = false;
		now_info.is_meet = false;
		now_info.is_pass = false;
		history_infos_.push_back(now_info);
		SaveHistoryInfos();
	}


	void LocalTargetGenerator::UpdateVirtualObstacles()
	{
		// construct virtural obstacles
		virtual_obstacles_.clear();
		virtual_obs_indx_.clear();
		static_obstacles_.clear();
		for (size_t i = 0; i < history_infos_.size(); i++)
		{
			if (history_infos_[i].event_slz.lane_id.road_id == hlmrlane_graph_.M.lane_info.id.road_id ||
				history_infos_[i].event_slz.lane_id.road_id == hlmrlane_graph_.HM.lane_info.id.road_id)
			{
				if (history_infos_[i].is_pass)
					continue;

				Box2d v_obs;
				zjlmap::XYZ v_obs_xy = Gmap_->xyz(history_infos_[i].event_slz);
				v_obs.x = v_obs_xy.x;
				v_obs.y = v_obs_xy.y;
				v_obs.z = v_obs_xy.z;
				v_obs.rlsl = history_infos_[i].event_slz;
				v_obs.speed = 0.0;
				v_obs.is_static = true;
				v_obs.is_virtual = true;
				v_obs.width = robot_.width;
				v_obs.length = robot_.length;
				v_obs.roleType = RoleType::UNKNOWN;

				if (!history_infos_[i].is_meet)
				{
					std::srand(time(0));
					double random_num = (std::rand() % (100) / (float)(101));
					LOG_INFO("random num: %.2f, prob: %.2f", random_num, history_infos_[i].prob);
					if (random_num > history_infos_[i].prob && history_infos_[i].speed_limit > 3.0)
						history_infos_[i].is_active = true;
					history_infos_[i].is_meet = true;
					history_infos_[i].prob = std::max(history_infos_[i].prob - 0.1, 0.0);
				}

				if (history_infos_[i].is_active) // exporation
				{
					virtual_obstacles_.push_back(v_obs);
					virtual_obs_indx_.push_back(i);
				}
				else
				{
					static_obstacles_.push_back(v_obs);
				}
			}
			else
			{
				history_infos_[i].is_meet = false;
				history_infos_[i].is_active = false;
				history_infos_[i].is_pass = false;
			}
		}
		SaveHistoryInfos();
	}

	void LocalTargetGenerator::UpdateVirtualObstaclesPassInfo()
	{
		for (size_t i = 0; i < virtual_obstacles_.size(); i++)
		{
			Box2d v_obs = virtual_obstacles_[i];
			size_t v_obs_indx = virtual_obs_indx_[i];
			if (v_obs.rlsl.lane_id == robot_.rlsl.lane_id && history_infos_[v_obs_indx].is_active)
			{
				double obs2ugv_dist = -SignofLane(hlmrlane_graph_.M.lane_info.id)*(v_obs.rlsl.s - robot_.rlsl.s);
				if (obs2ugv_dist < -10.0)
				{
					history_infos_[v_obs_indx].is_pass = true;
					history_infos_[v_obs_indx].is_meet = false;
					history_infos_[v_obs_indx].is_active = false;
					history_infos_[v_obs_indx].speed_pass = history_infos_[v_obs_indx].speed_pass
						+ (history_infos_[v_obs_indx].speed_limit - history_infos_[v_obs_indx].speed_pass) / 3.0;

					virtual_obstacles_.erase(std::begin(virtual_obstacles_) + i);
					virtual_obs_indx_.erase(std::begin(virtual_obs_indx_) + i);
					break;
				}
			}
		}
		SaveHistoryInfos();
	}

} // namespace localPlanner
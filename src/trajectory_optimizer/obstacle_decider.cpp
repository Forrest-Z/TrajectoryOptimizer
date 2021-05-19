#include "obstacle_decider.h"

namespace PlanningOptimizer
{
ObstacleDecider::ObstacleDecider(const std::vector<localPlanner::Box2d>& obstacle_list, const std::shared_ptr<zjlmap::Map>& map_data)
    : obstacle_list_(obstacle_list), map_data_(map_data)
{
}

// void ObstacleDecider::CalculateSLZAndDistanceOfObstacles(const TrajectoryPoint& ego_vehicle_state)
//{
//	for (size_t i = 0; i < obstacle_list_.size(); i++)
//	{
//		zjlmap::SLZ obstacle_center_slz;
//		zjlmap::XYZ obstacle_center_xyz = zjlmap::XYZ(obstacle_list_[i].x, obstacle_list_[i].y, obstacle_list_[i].z);
//		zjlmap::ErrorCode ec_query_obstacle = map_data_->find_slz_global(obstacle_center_xyz, obstacle_center_slz);
//		obstacle_list_[i].slz = obstacle_center_slz;
//
//		double delta_x = ego_vehicle_state.path_point.x - obstacle_list_[i].x;
//		double delta_y = ego_vehicle_state.path_point.y - obstacle_list_[i].y;
//		double distance_to_obstacle = sqrt(delta_x * delta_x + delta_y * delta_y);
//		obstacle_list_[i].distance = distance_to_obstacle;
//	}
//	return;
//}

bool ObstacleDecider::IsRelevantObstacles(const localPlanner::Box2d& ego_vehicle, const std::vector<localPlanner::TargetNode> targets,
                                          localPlanner::Box2d obstacle)
{
    if (!(obstacle.rlsl.lane_id == ego_vehicle.rlsl.lane_id) && !(obstacle.rlsl.lane_id == targets[0].rlsl.lane_id))
    {
        return false;
    }

    return true;
}

void ObstacleDecider::ProcessLaneChangeObstacles(const localPlanner::Box2d& ego_vehicle,
                                                 const std::vector<localPlanner::TargetNode> targets, TargetLine& target_line,
                                                 PathBoundary& path_boundary)
{
    for (size_t i = 0; i < obstacle_list_.size(); i++)
    {
        bool is_self_lane_obstacle = false;
        if (targets[0].lane_type == localPlanner::LaneType::LEFT)
        {
            if (obstacle_list_[i].rlsl.lane_id == ego_vehicle.rlsl.lane_id)  // process the self lane obstacles
            {
                is_self_lane_obstacle = true;
                ObstacleCorners self_lane_obstacle_corners;
                GetBoundingBoxOfObstacle(ego_vehicle.rlsl.lane_id, obstacle_list_[i], target_line, self_lane_obstacle_corners);
                PathBoundaryUpdateByObstacles(ego_vehicle.rlsl.lane_id, self_lane_obstacle_corners, targets[0].lane_type,
                                              is_self_lane_obstacle, path_boundary);
            }
            else if (obstacle_list_[i].rlsl.lane_id == targets[0].rlsl.lane_id)  // process the left lane obstacles
            {
                is_self_lane_obstacle = false;
                ObstacleCorners left_lane_obstacle_corners;
                GetBoundingBoxOfObstacle(ego_vehicle.rlsl.lane_id, obstacle_list_[i], target_line, left_lane_obstacle_corners);
                PathBoundaryUpdateByObstacles(ego_vehicle.rlsl.lane_id, left_lane_obstacle_corners, targets[0].lane_type,
                                              is_self_lane_obstacle, path_boundary);
            }
        }
        else if (targets[0].lane_type == localPlanner::LaneType::RIGHT)
        {
            if (obstacle_list_[i].rlsl.lane_id == ego_vehicle.rlsl.lane_id)  // process the self lane obstacles
            {
                is_self_lane_obstacle = true;
                ObstacleCorners self_lane_obstacle_corners;
                GetBoundingBoxOfObstacle(ego_vehicle.rlsl.lane_id, obstacle_list_[i], target_line, self_lane_obstacle_corners);
                PathBoundaryUpdateByObstacles(ego_vehicle.rlsl.lane_id, self_lane_obstacle_corners, targets[0].lane_type,
                                              is_self_lane_obstacle, path_boundary);
            }
            else if (obstacle_list_[i].rlsl.lane_id == targets[0].rlsl.lane_id)  // process the right lane obstacles
            {
                is_self_lane_obstacle = false;
                ObstacleCorners right_lane_obstacle_corners;
                GetBoundingBoxOfObstacle(ego_vehicle.rlsl.lane_id, obstacle_list_[i], target_line, right_lane_obstacle_corners);
                PathBoundaryUpdateByObstacles(ego_vehicle.rlsl.lane_id, right_lane_obstacle_corners, targets[0].lane_type,
                                              is_self_lane_obstacle, path_boundary);
            }
        }
    }

    return;
}

void ObstacleDecider::GetBoundingBoxOfObstacle(const zjlmap::LaneId& target_lane_id, const localPlanner::Box2d& obstacle,
                                               TargetLine& target_line, ObstacleCorners& obstacle_corners)
{
    // get the corners of the obstacle
    std::vector<zjlmap::XYZ> corners_xyz;
    std::vector<zjlmap::SLZ> corners_slz;
    GetCornersOfObstacle(target_lane_id, obstacle, corners_xyz, corners_slz);

    if (corners_xyz.size() == 0 || corners_slz.size() == 0)
    {
        std::cout << "[ERROR] the corners size is 0! " << std::endl;
        return;
    }

    std::cout << "[Debug] XYZ corners of obstacle: " << std::endl;
    for (size_t j = 0; j < corners_xyz.size(); j++)
    {
        std::cout << "corner " << j << " x: " << corners_xyz[j].x << " y: " << corners_xyz[j].y << std::endl;
    }

    // find the road-based road_l_min, road_l_max
    obstacle_corners.road_l_min = corners_slz[0].l;
    obstacle_corners.road_l_max = corners_slz[0].l;
    for (size_t i = 1; i < corners_slz.size(); i++)
    {
        obstacle_corners.road_l_min = std::min(obstacle_corners.road_l_min, corners_slz[i].l);
        obstacle_corners.road_l_max = std::max(obstacle_corners.road_l_max, corners_slz[i].l);
    }
    std::cout << "[Debug] The obstacle corner road_l_min: " << obstacle_corners.road_l_min << " road_l_max: " << obstacle_corners.road_l_max
              << std::endl;

    // find the l_min, l_max, s_min, s_max
    zjlmap::TracePoint obstacle_corner_trace_point0;
    target_line.GetTargetTracePoint(corners_slz[0], obstacle_corner_trace_point0);
    int sign0 = abs(corners_slz[0].l) > abs(obstacle_corner_trace_point0.l) ? 1 : -1;
    corners_slz[0].l = sign0 * (abs(corners_slz[0].l - obstacle_corner_trace_point0.l));
    obstacle_corners.l_min = corners_slz[0].l;
    obstacle_corners.l_max = corners_slz[0].l;
    obstacle_corners.s_min = corners_slz[0].s;
    obstacle_corners.s_max = corners_slz[0].s;

    std::cout << "[Debug] SLZ corners of obstacle: " << std::endl;
    for (size_t j = 1; j < corners_slz.size(); j++)
    {
        // convert to lane-based frenet frame
        zjlmap::TracePoint obstacle_corner_trace_point;
        target_line.GetTargetTracePoint(corners_slz[j], obstacle_corner_trace_point);
        int signj = abs(corners_slz[j].l) > abs(obstacle_corner_trace_point.l) ? 1 : -1;
        corners_slz[j].l = signj * (abs(corners_slz[j].l - obstacle_corner_trace_point.l));
        std::cout << "corner " << j << " road_id: " << corners_slz[j].lane_id.road_id << " local_id: " << corners_slz[j].lane_id.local_id
                  << " s: " << corners_slz[j].s << " l: " << corners_slz[j].l << std::endl;

        obstacle_corners.l_min = std::min(obstacle_corners.l_min, corners_slz[j].l);
        obstacle_corners.l_max = std::max(obstacle_corners.l_max, corners_slz[j].l);
        obstacle_corners.s_min = std::min(obstacle_corners.s_min, corners_slz[j].s);
        obstacle_corners.s_max = std::max(obstacle_corners.s_max, corners_slz[j].s);
    }
    std::cout << "[Debug] The obstacle corner l_min: " << obstacle_corners.l_min << " l_max: " << obstacle_corners.l_max
              << " s_min: " << obstacle_corners.s_min << " s_max: " << obstacle_corners.s_max << std::endl;

    return;
}

void ObstacleDecider::GetCornersOfObstacle(const zjlmap::LaneId& target_lane_id, const localPlanner::Box2d& obstacle,
                                           std::vector<zjlmap::XYZ>& corners_xyz, std::vector<zjlmap::SLZ>& corners_slz)
{
    double cos_heading = cos(obstacle.theta);
    double sin_heading = sin(obstacle.theta);
    double half_length = obstacle.length / 2.0;
    double half_width = obstacle.width / 2.0;
    double dx1 = cos_heading * half_length;
    double dy1 = sin_heading * half_length;
    double dx2 = sin_heading * half_width;
    double dy2 = -cos_heading * half_width;
    std::cout << "[Debug] cos_heading: " << cos_heading << " sin_heading: " << sin_heading << " dx1: " << dx1 << " dy1: " << dy1
              << " dx2: " << dx2 << " dy2: " << dy2 << std::endl;

    corners_xyz.push_back(zjlmap::XYZ(obstacle.x + dx1 + dx2, obstacle.y + dy1 + dy2, 0.0));
    corners_xyz.push_back(zjlmap::XYZ(obstacle.x + dx1 - dx2, obstacle.y + dy1 - dy2, 0.0));
    corners_xyz.push_back(zjlmap::XYZ(obstacle.x - dx1 - dx2, obstacle.y - dy1 - dy2, 0.0));
    corners_xyz.push_back(zjlmap::XYZ(obstacle.x - dx1 + dx2, obstacle.y - dy1 + dy2, 0.0));

    for (size_t i = 0; i < corners_xyz.size(); i++)
    {
        zjlmap::SLZ slz_point;
        if (map_data_->is_point_on_road(target_lane_id.road_id, corners_xyz[i]))
        {
            zjlmap::ErrorCode ec_query_corner_pt = map_data_->calc_slz_with_road_id(corners_xyz[i], target_lane_id.road_id, slz_point);
            if (ec_query_corner_pt == zjlmap::ErrorCode::kOK)
            {
                corners_slz.push_back(slz_point);
            }
        }
        else
        {
            zjlmap::ErrorCode ec_query_corner_pt = map_data_->find_slz_global(corners_xyz[i], slz_point);
            if (ec_query_corner_pt == zjlmap::ErrorCode::kOK)
            {
                corners_slz.push_back(slz_point);
            }
        }
    }

    return;
}

void ObstacleDecider::PathBoundaryUpdateByObstacles(const zjlmap::LaneId& target_lane_id, const ObstacleCorners& obstacle_corners,
                                                    const localPlanner::LaneType& lane_type, const bool& is_self_lane_obstacle,
                                                    PathBoundary& path_boundary)
{
    if (lane_type == localPlanner::LaneType::LEFT)
    {
        if (is_self_lane_obstacle)  // shrink right path boundary for self lane in left lane change situation
        {
            for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
            {
                // shrink the right path boundary
                if (path_boundary.road_based_sl_boundary[i].first.s >= (obstacle_corners.s_min - s_buffer) &&
                    path_boundary.road_based_sl_boundary[i].first.s <= (obstacle_corners.s_max + s_buffer))
                {
                    path_boundary.lane_based_sl_boundary[i].second = obstacle_corners.l_min - l_buffer;
                    if (target_lane_id.local_id > 0)
                    {
                        path_boundary.road_based_sl_boundary[i].second.l = obstacle_corners.road_l_min - l_buffer;
                    }
                    else
                    {
                        path_boundary.road_based_sl_boundary[i].second.l = obstacle_corners.road_l_max + l_buffer;
                    }
                }
            }
        }
        else  // shrink the left path boundary for target left lane
        {
            for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
            {
                // shrink the left path boundary
                if (path_boundary.road_based_sl_boundary[i].first.s >= (obstacle_corners.s_min - s_buffer) &&
                    path_boundary.road_based_sl_boundary[i].first.s <= (obstacle_corners.s_max + s_buffer))
                {
                    path_boundary.lane_based_sl_boundary[i].first = obstacle_corners.l_max + l_buffer;
                    if (target_lane_id.local_id > 0)
                    {
                        path_boundary.road_based_sl_boundary[i].first.l = obstacle_corners.road_l_max + l_buffer;
                    }
                    else
                    {
                        path_boundary.road_based_sl_boundary[i].first.l = obstacle_corners.road_l_min - l_buffer;
                    }
                }
            }
        }
    }
    else if (lane_type == localPlanner::LaneType::RIGHT)
    {
        if (is_self_lane_obstacle)  // shrink left path boundary for self lane in right lane change situation
        {
            for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
            {
                // shrink the left path boundary
                if (path_boundary.road_based_sl_boundary[i].first.s >= (obstacle_corners.s_min - s_buffer) &&
                    path_boundary.road_based_sl_boundary[i].first.s <= (obstacle_corners.s_max + s_buffer))
                {
                    path_boundary.lane_based_sl_boundary[i].first = obstacle_corners.l_max + l_buffer;
                    if (target_lane_id.local_id > 0)
                    {
                        path_boundary.road_based_sl_boundary[i].first.l = obstacle_corners.road_l_max + l_buffer;
                    }
                    else
                    {
                        path_boundary.road_based_sl_boundary[i].first.l = obstacle_corners.road_l_min - l_buffer;
                    }
                }
            }
        }
        else  // shrink the right path boundary for target right lane
        {
            for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
            {
                // shrink the right path boundary
                if (path_boundary.road_based_sl_boundary[i].first.s >= (obstacle_corners.s_min - s_buffer) &&
                    path_boundary.road_based_sl_boundary[i].first.s <= (obstacle_corners.s_max + s_buffer))
                {
                    path_boundary.lane_based_sl_boundary[i].second = obstacle_corners.l_min - l_buffer;
                    if (target_lane_id.local_id > 0)
                    {
                        path_boundary.road_based_sl_boundary[i].second.l = obstacle_corners.road_l_min - l_buffer;
                    }
                    else
                    {
                        path_boundary.road_based_sl_boundary[i].second.l = obstacle_corners.road_l_max + l_buffer;
                    }
                }
            }
        }
    }

    return;
}

void ObstacleDecider::LaneBorrowUpdateByStaticObstacles(const zjlmap::LaneId& target_lane_id, const double& road_l_min,
                                                        const double& road_l_max, const double& l_min, const double& l_max,
                                                        const double& s_min, const double& s_max, TargetLine& target_line,
                                                        PathBoundary& path_boundary)
{
    zjlmap::LaneLinkage neighbour_lane_linkage;
    map_data_->query_lane_linkage(target_lane_id, neighbour_lane_linkage);
    std::cout << "[Debug] left neighbour lane road id: " << neighbour_lane_linkage.left_neighbor.road_id
              << " local_id: " << neighbour_lane_linkage.left_neighbor.local_id << std::endl;

    if (neighbour_lane_linkage.left_neighbor.road_id != -1)
    {
        for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
        {
            // shrink the right path boundary
            if (path_boundary.road_based_sl_boundary[i].first.s >= (s_min - s_buffer) &&
                path_boundary.road_based_sl_boundary[i].first.s <= (s_max + s_buffer))
            {
                std::cout << "[Debug] inside the s range index: " << i << std::endl;
                path_boundary.lane_based_sl_boundary[i].second = l_min - l_buffer;
                std::cout << "[Debug] inside the s range lane-based right l: " << path_boundary.lane_based_sl_boundary[i].second
                          << std::endl;
                if (target_lane_id.local_id > 0)
                {
                    path_boundary.road_based_sl_boundary[i].second.l = road_l_min - l_buffer;
                }
                else
                {
                    path_boundary.road_based_sl_boundary[i].second.l = road_l_max + l_buffer;
                }
                std::cout << "[Debug] inside the s range road-based right l: " << path_boundary.road_based_sl_boundary[i].second.l
                          << std::endl;
            }

            // extend the left path boundary
            zjlmap::SLZ pos_left = path_boundary.road_based_sl_boundary[i].first;
            pos_left.lane_id = neighbour_lane_linkage.left_neighbor;
            double left_neighbour_lane_width;
            map_data_->query_lane_width_at(pos_left, left_neighbour_lane_width);

            // std::cout << "[Debug] The left neighbour lane width: " << left_neighbour_lane_width << std::endl;
            path_boundary.lane_based_sl_boundary[i].first -= left_neighbour_lane_width;
            if (target_lane_id.local_id > 0)
            {
                path_boundary.road_based_sl_boundary[i].first.l -= left_neighbour_lane_width;
            }
            else
            {
                path_boundary.road_based_sl_boundary[i].first.l += left_neighbour_lane_width;
            }
        }
    }
    else if (neighbour_lane_linkage.right_neighbor.road_id != -1)
    {
        for (size_t i = 0; i < path_boundary.lane_based_sl_boundary.size(); i++)
        {
            // shrink the left path boundary
            if (path_boundary.road_based_sl_boundary[i].first.s >= (s_min - s_buffer) &&
                path_boundary.road_based_sl_boundary[i].first.s <= (s_max + s_buffer))
            {
                path_boundary.lane_based_sl_boundary[i].first = l_max + l_buffer;
                if (target_lane_id.local_id > 0)
                {
                    path_boundary.road_based_sl_boundary[i].first.l = road_l_max + l_buffer;
                }
                else
                {
                    path_boundary.road_based_sl_boundary[i].first.l = road_l_min - l_buffer;
                }
            }

            // extend the right path boundary
            zjlmap::SLZ pos_right = path_boundary.road_based_sl_boundary[i].second;
            pos_right.lane_id = neighbour_lane_linkage.right_neighbor;
            double right_neighbour_lane_width;
            map_data_->query_lane_width_at(pos_right, right_neighbour_lane_width);
            // std::cout << "[Debug] The right neighbour lane width: " << right_neighbour_lane_width << std::endl;
            path_boundary.lane_based_sl_boundary[i].second += right_neighbour_lane_width;
            if (target_lane_id.local_id > 0)
            {
                path_boundary.road_based_sl_boundary[i].second.l += right_neighbour_lane_width;
            }
            else
            {
                path_boundary.road_based_sl_boundary[i].second.l -= right_neighbour_lane_width;
            }
        }
    }
    else
    {
        // [Todo] Block by static obstacle - need to stop
    }

    return;
}
//
//// return true: can lane change; return false: canNOT lane change
// bool ObstacleDecider::LaneChangePathProcessObstacle(const TrajectoryPoint& ego_vehicle_state, const zjlmap::SLZ& start_point_slz, const
// zjlmap::LaneId& next_lane_id, bool& need_estop)
//{
//	for (size_t i = 0; i < obstacle_list_.size(); i++)
//	{
//		if (obstacle_list_[i].rlsl.lane_id == next_lane_id && obstacle_list_[i].roleType == RoleType::VEHICLE
//			&& obstacle_list_[i].is_static == false)
//		{
//			if (abs(obstacle_list_[i].theta - ego_vehicle_state.path_point.theta) > M_PI / 2.0)
//			{
//				if ((next_lane_id.local_id > 0 && obstacle_list_[i].rlsl.s < start_point_slz.s) ||
//					(next_lane_id.local_id < 0 && obstacle_list_[i].rlsl.s > start_point_slz.s))
//				{
//					std::cout << "[Debug] Cannot lane change cause wrong direction vehicle. " << std::endl;
//					need_estop = false;
//					return false;
//				}
//				else
//				{
//					std::cout << "[Debug] Can lane change cause wrong direction vehicle is behind the ego car. " << std::endl;
//					need_estop = false;
//					return true;
//				}
//			}
//			/*else if (obstacle_list_[i].distance < safe_lane_change_dis_threshold)
//			{
//				std::cout << "[Debug] Cannot lane change cause too close dynamic obstacle. " << std::endl;
//				need_estop = true;
//				return true;
//			}*/
//
//		}
//		else if (obstacle_list_[i].rlsl.lane_id == next_lane_id && obstacle_list_[i].is_static == true)
//		{
//			if ((next_lane_id.local_id > 0 && obstacle_list_[i].rlsl.s < start_point_slz.s) ||
//				(next_lane_id.local_id < 0 && obstacle_list_[i].rlsl.s > start_point_slz.s))
//			{
//				std::cout << "[Debug] Cannot lane change cause front static obstacle. " << std::endl;
//				need_estop = false;
//				return false;
//			}
//		}
//	}
//	std::cout << "[Debug] lane change obstacle process need_estop: " << need_estop << std::endl;
//
//	return true;
//}
//
//// return true: need lane change to avoid static obstacle and wrong direction vehicle
// bool ObstacleDecider::SelfLanePathProcessObstacle(const TrajectoryPoint& ego_vehicle_state, const zjlmap::SLZ& start_point_slz, const
// zjlmap::LaneId& curr_lane_id)
//{
//	zjlmap::JunctionId junction_id;
//	zjlmap::ErrorCode query_junction_id = map_data_->is_road_in_junction(start_point_slz.lane_id.road_id, junction_id);
//	if (junction_id != -1)
//	{
//		std::cout << "[Debug] The ego vehicle is current at the junction, cannot lane change. " << std::endl;
//		return false;
//	}
//
//
//	for (size_t i = 0; i < obstacle_list_.size(); i++)
//	{
//		localPlanner::Box2d obstacle = obstacle_list_[i];
//		double delta_x = obstacle.x - ego_vehicle_state.path_point.x;
//		double delta_y = obstacle.y - ego_vehicle_state.path_point.y;
//		double theta_to_obstacle = atan2(delta_y, delta_x);
//		double angle_diff = std::abs(theta_to_obstacle - ego_vehicle_state.path_point.theta);
//		/*if (obstacle.distance > static_obstacle_lane_change_dis_lowerbound && obstacle.distance <
//static_obstacle_lane_change_dis_upperbound
//			&& obstacle.is_static == true && angle_diff < M_PI / 4.0 && obstacle_list_[i].roleType != RoleType::PEDESTRIAN)
//		{
//			std::cout << "[Debug] 111lane change obstacle info road id: " << obstacle.slz.lane_id.road_id << " section_idx: " <<
//obstacle.slz.lane_id.section_idx << " local_id: " << 				obstacle.slz.lane_id.local_id << " dis: " << obstacle.distance << " roleType: " <<
//obstacle.roleType << " id: " << obstacle.id << std::endl; 			if (obstacle.slz.lane_id == curr_lane_id && obstacle.roleType !=
//RoleType::PEDESTRIAN)
//			{
//				std::cout << "[Debug] lane change obstacle info road id: " << obstacle.slz.lane_id.road_id << " section_idx: " <<
//obstacle.slz.lane_id.section_idx << " local_id: " << 					obstacle.slz.lane_id.local_id << " dis: " << obstacle.distance << " roleType: " <<
//obstacle.roleType << " id: " << obstacle.id << std::endl; 				return true;
//			}
//
//		}*/
//		if (abs(obstacle_list_[i].theta - ego_vehicle_state.path_point.theta) > M_PI / 4.0 && obstacle_list_[i].rlsl.lane_id == curr_lane_id
//			&& obstacle.is_static != true && obstacle_list_[i].roleType != RoleType::PEDESTRIAN)
//		{
//			if ((curr_lane_id.local_id > 0 && obstacle_list_[i].rlsl.s < start_point_slz.s) ||
//				(curr_lane_id.local_id < 0 && obstacle_list_[i].rlsl.s > start_point_slz.s))
//			{
//				std::cout << "[Debug] Need to lane change cause wrong direction vehicle. " << std::endl;
//				return true;
//			}
//		}
//	}
//	return false;
//}

bool ObstacleDecider::SpeedProcessDynamicObstacle(const TrajectoryPoint& ego_vehicle_state)
{
    for (size_t i = 0; i < obstacle_list_.size(); i++)
    {
        double delta_x = obstacle_list_[i].x - ego_vehicle_state.path_point.x;
        double delta_y = obstacle_list_[i].y - ego_vehicle_state.path_point.y;
        double theta_to_obstacle = atan2(delta_y, delta_x);
        double angle_diff = std::abs(theta_to_obstacle - ego_vehicle_state.path_point.theta);
        std::cout << "theta_to_obstacle: " << theta_to_obstacle << std::endl;
        std::cout << "ego theta: " << ego_vehicle_state.path_point.theta << std::endl;
        std::cout << "angle diff: " << angle_diff << std::endl;

        // if front too close obstacle then stop
        /*if (angle_diff < M_PI / 4.0 && obstacle_list_[i].distance < pedestrain_dis_threshold)
        {
            std::cout << "[Debug] Enter too close front obstacle situation. " << std::endl;
            return true;
        }*/
        /*if (obstacle_list_[i].roleType == RoleType::PEDESTRIAN && angle_diff < M_PI / 4.0 && obstacle_list_[i].distance <
        pedestrain_dis_threshold)
        {
            std::cout << "[Debug] Enter too close front obstacle situation. " << std::endl;
            return true;
        }*/
    }

    return false;
}

}  // namespace PlanningOptimizer
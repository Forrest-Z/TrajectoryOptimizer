#include "trajectory_planner.h"

namespace PlanningOptimizer
{
TrajectoryPlanner::TrajectoryPlanner()
{
}

double QuaternionToEulerAngle(double w, double x, double y, double z)
{
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

bool TrajectoryPlanner::Init(const std::shared_ptr<zjlmap::Map>& map_data)
{
    map_data_ = map_data;

    // Init path planner and speed planner
    if (!path_planner_handle.Init(map_data_))
    {
        std::cout << "[ERROR] The path planner init process is failed! " << std::endl;
        return false;
    }

    if (!speed_planner_handle.Init(map_data_))
    {
        std::cout << "[ERROR] The speed planner init process is failed! " << std::endl;
        return false;
    }
    return true;
}

bool TrajectoryPlanner::RoutingPlan(const XYZ& routing_start_point_xyz, const XYZ& routing_end_point_xyz)
{
    std::cout << "Routing start ... " << std::endl;
    SLZ routing_start_point_slz;
    // XYZ routing_start_point_xyz = { 784514.04967285, 3352802.09302979, 0.0 };
    map_data_->find_slz_global(routing_start_point_xyz, routing_start_point_slz);
    Anchor start_anchor = map_data_->create_anchor("start", routing_start_point_slz);

    SLZ routing_end_point_slz;
    // XYZ routing_end_point_xyz = { 784725.80615234, 3352898.41951172, 0.0 };
    map_data_->find_slz_global(routing_end_point_xyz, routing_end_point_slz);
    Anchor end_anchor = map_data_->create_anchor("end", routing_end_point_slz);

    AnchorArray routing_anchor_array;
    routing_anchor_array.length = 2;
    routing_anchor_array.anchor_array = (Anchor*)malloc(sizeof(Anchor) * 2);
    routing_anchor_array.anchor_array[0] = start_anchor;
    routing_anchor_array.anchor_array[1] = end_anchor;

    LaneId lane_id_extend(24, 0, -2);
    SLZ routing_extend_point_slz(lane_id_extend, 12.0, -5.1, 0.0);
    Anchor extend_anchor = map_data_->create_anchor("extend", routing_extend_point_slz);
    map_data_->add_anchor(extend_anchor, routing_anchor_array);

    AnchorArray routing_waypoint_list;
    navigation_data.routing_result.lane_id_vec.clear();
    map_data_->plan_route(start_anchor, routing_waypoint_list, end_anchor, navigation_data.routing_result);  // A* navigation algorithm
    std::cout << "[Debug path planner] the routing result: " << std::endl;
    for (size_t i = 0; i < navigation_data.routing_result.lane_id_vec.size(); i++)
    {
        std::cout << " road_id : " << navigation_data.routing_result.lane_id_vec[i].road_id
                  << " section index : " << navigation_data.routing_result.lane_id_vec[i].section_idx
                  << " local_id : " << navigation_data.routing_result.lane_id_vec[i].local_id << std::endl;
    }
    std::cout << "routing distance = " << navigation_data.routing_result.length << std::endl;

    return true;
}

bool TrajectoryPlanner::Process(const std::vector<localPlanner::Box2d>& obstacles, const localPlanner::Box2d& robot,
                                  std::vector<localPlanner::TargetNode> targets, std::vector<TrajectoryPoint>& planning_trajectory_data)
{
	// debug the decision filtered obstacles  
	std::cout << "[Debug] The decision filtered obstacles debug: " << std::endl; 
	for (size_t i = 0; i < obstacles.size(); i++)
	{
		std::cout << "obstacle " << i << " id: " << obstacles[i].id << " road_id: " << obstacles[i].rlsl.lane_id.road_id << " local_id: " << obstacles[i].rlsl.lane_id.local_id <<
			" s: " << obstacles[i].rlsl.s << " l: " << obstacles[i].rlsl.l << " is_static: " << obstacles[i].is_static << std::endl; 
	}
	
	// create obstacle object  
	ObstacleDecider obstacle_decider_handle(obstacles, map_data_);
	
	// sort the targets by driving probability 
	auto comp = [](localPlanner::TargetNode first, localPlanner::TargetNode second) {
		return first.driving_prob > second.driving_prob;
	};
	std::sort(targets.begin(), targets.end(), comp);
	for (size_t i = 0; i < targets.size(); i++)
	{
		std::cout << "[sorted] target " << i << " road_id: " << targets[i].rlsl.lane_id.road_id << " local_id: " << targets[i].rlsl.lane_id.local_id << " s: " << targets[i].rlsl.s <<
			" road_l: " << targets[i].rlsl.l << " lane_l: " << targets[i].sl.l << " laneType: " << targets[i].lane_type << " dis_to_ego: " <<  targets[i].sl.s << 
			" probo: " << targets[i].driving_prob << std::endl;
		std::cout << "target " << i << " x: " << targets[i].pose.x << " y: " << targets[i].pose.y << std::endl;
	}

    // [delete it!] change target for debug 
    if (targets[0].rlsl.lane_id.road_id == 17 && targets[0].rlsl.lane_id.local_id == -1)
    {
        targets[0].rlsl.lane_id.local_id = -2;
        targets[0].lane_type = localPlanner::LaneType::RIGHT;
    }
    else if (targets[0].rlsl.lane_id.road_id == 17 && targets[0].rlsl.lane_id.local_id == -2)
    {
        targets[0].rlsl.lane_id.local_id = -1;
        targets[0].lane_type = localPlanner::LaneType::LEFT;
    }

	// path planning part
    struct timeb path_start_time;
    struct timeb path_end_time;
    ftime(&path_start_time);
    std::vector<CartesianPathPoint> planning_path_data;
	if (!path_planner_handle.Process(robot, targets, obstacle_decider_handle, planning_path_data))
	{
		std::cout << "[ERROR] path planner process step is failed! " << std::endl;
		return false;
	}
    ftime(&path_end_time);
    int64_t path_time_cost = (path_end_time.time - path_start_time.time) * 1000 + (path_end_time.millitm - path_start_time.millitm);

    std::cout << "Path Time cost is: " << path_time_cost << std::endl;

    if (planning_path_data.size() == 0)
	{
		std::cout << "[ERROR] planning path data size is 0! " << std::endl;
		return false;
	}

	planning_trajectory_data.resize(planning_path_data.size());
	for (size_t i = 0; i < planning_trajectory_data.size(); i++)
	{
		planning_trajectory_data[i].path_point = planning_path_data[i];
	}

    // speed planning part
    if (!speed_planner_handle.Process(robot, targets, obstacle_decider_handle, planning_trajectory_data))
    {
    	std::cout << "[ERROR] speed planner process step is failed! " << std::endl;
    	return false;
    }

    // // New ST speed planner
    // if (!st_speed_planner_handle.Process(robot, targets, obstacle_decider_handle, planning_trajectory_data))
    // {
    //     std::cout << "[ERROR] speed planner process step is failed! ";
    //     return false;
    // }

    //// decide if need emergency stop 
	//if (speed_planner_handle.is_need_estop == true)
	//{
	//	std::cout << "[Debug] speed estop triggered. ";
	//	trajectory_estop = true;
	//}
	//else if (path_planner_handle.path_driven_estop == true)
	//{
	//	std::cout << "[Debug] path estop triggered. ";
	//	trajectory_estop = true;
	//}
	//else
	//{
	//	trajectory_estop = false;
	//}

	return true;
}

}  // namespace PlanningOptimizer
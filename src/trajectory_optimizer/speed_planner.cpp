#include "speed_planner.h"

namespace PlanningOptimizer
{
SpeedPlanner::SpeedPlanner() {}

bool SpeedPlanner::Init(const std::shared_ptr<Map>& map_data)
{
    map_data_ = map_data;
    forward_curvature_length = 20.0;
    max_centric_acc_limit = 2.0;
    lowest_speed = 2.5;
    highest_speed = 35.0;

    return true;
}

bool SpeedPlanner::Process(const localPlanner::Box2d& robot, const std::vector<localPlanner::TargetNode> targets,
                           ObstacleDecider& obstacle_decider_handle, std::vector<TrajectoryPoint>& planning_trajectory_data)
{
    //// decide if need emergency stop before dynamic obstacle
    // if (obstacle_decider_handle.SpeedProcessDynamicObstacle(planning_start_point))
    //{
    //	std::cout << "[Debug] Enter emergency stop cause dynamic obstacle. " << std::endl;
    //	is_need_estop = true;
    //	return true;
    //}
    // else
    //{
    //	is_need_estop = false;
    //}

    // Get speed ref from map

    // Get speed ref from curvature
    double desired_delta = std::atan(planning_trajectory_data[0].path_point.kappa * wheel_base);
    double speed_base_on_curvature = std::min(std::exp(-10.0 * (fabs(desired_delta) - 0.2)) + 3.0, 10.0);

    // Get speed ref from look ahead dis
    double lon_dist = targets[0].sl.s;
    std::cout << "[Debug] lon_dist: " << lon_dist << std::endl;
    double speed_base_on_lookaheaddist = std::max(0.0, std::min(std::sqrt(lon_dist * 5.0) - 2.0, 10.0));

    double desired_speed = std::min(speed_base_on_curvature, speed_base_on_lookaheaddist);

    if (desired_speed > 0.0)
    {
        desired_speed = 0.7 * pre_desired_speed + 0.3 * desired_speed;
    }
    pre_desired_speed = desired_speed;

    std::cout << "[Debug] desired speed: " << desired_speed << std::endl;

    for (size_t i = 0; i < planning_trajectory_data.size(); i++)
    {
        planning_trajectory_data[i].v = desired_speed;
    }

    return true;
}

/*
double SpeedPlanner::GetSpeedRefFromMap(const std::vector<TrajectoryPoint>& planning_trajectory_data)
{
    double speed_ref_from_traffic_rule = 0.0;

    for (size_t i = 0; i < planning_trajectory_data.size(); i++)
    {
        ErrorCode query_lane_speed_ec = map_data_->query_lane_speed_at(const LaneId &id, const RoadS &s, speed_ref_from_traffic_rule);

    }
}
*/

double SpeedPlanner::GetSpeedRefFromCurvature(const std::vector<TrajectoryPoint>& planning_trajectory_data)
{
    double max_curvature = 0.000001;
    double accumulate_dis = 0.0;
    int iter = 0;
    forward_curvature_length = std::min(planning_trajectory_data.back().path_point.accumulate_dis, forward_curvature_length);
    while (accumulate_dis < forward_curvature_length)
    {
        if (planning_trajectory_data[iter].path_point.kappa > max_curvature)
        {
            max_curvature = planning_trajectory_data[iter].path_point.kappa;
        }
        iter++;
        accumulate_dis += planning_trajectory_data[iter].path_point.accumulate_dis;
    }

    double speed_ref_from_curvature = std::sqrt(max_centric_acc_limit / max_curvature);

    return speed_ref_from_curvature;
}

}  // namespace PlanningOptimizer
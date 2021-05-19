#pragma once

#include <math.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>

#include "header.h"
#include "planning_struct.h"
#include "target_line.h"

#define M_PI 3.14159265358979323846  // pi

struct ObstacleCorners
{
    double road_l_min;
    double road_l_max;
    double l_min;  // lane based l corrd
    double l_max;
    double s_min;
    double s_max;
};

namespace PlanningOptimizer
{
class ObstacleDecider
{
   public:
    ObstacleDecider(const std::vector<localPlanner::Box2d>& obstacle_list, const std::shared_ptr<zjlmap::Map>& map_data);
    ~ObstacleDecider(){};

    // void CalculateSLZAndDistanceOfObstacles(const TrajectoryPoint& ego_vehicle_state);
    bool IsRelevantObstacles(const localPlanner::Box2d& ego_vehicle, const std::vector<localPlanner::TargetNode> targets,
                             localPlanner::Box2d obstacle);
    void ProcessLaneChangeObstacles(const localPlanner::Box2d& ego_vehicle, const std::vector<localPlanner::TargetNode> targets,
                                    TargetLine& target_line, PathBoundary& path_boundary);
    void GetCornersOfObstacle(const zjlmap::LaneId& target_lane_id, const localPlanner::Box2d& obstacle,
                              std::vector<zjlmap::XYZ>& corners_xyz, std::vector<zjlmap::SLZ>& corners_slz);
    void GetBoundingBoxOfObstacle(const zjlmap::LaneId& target_lane_id, const localPlanner::Box2d& obstacle, TargetLine& target_line,
                                  ObstacleCorners& obstacle_corners);
    void LaneBorrowUpdateByStaticObstacles(const zjlmap::LaneId& target_lane_id, const double& road_l_min, const double& road_l_max,
                                           const double& l_min, const double& l_max, const double& s_min, const double& s_max,
                                           TargetLine& target_line, PathBoundary& path_boundary);
    bool LaneChangePathProcessObstacle(const TrajectoryPoint& ego_vehicle_state, const zjlmap::SLZ& start_point_slz,
                                       const zjlmap::LaneId& next_lane_id, bool& need_estop);
    bool SpeedProcessDynamicObstacle(const TrajectoryPoint& ego_vehicle_state);
    bool SelfLanePathProcessObstacle(const TrajectoryPoint& ego_vehicle_state, const zjlmap::SLZ& start_point_slz,
                                     const zjlmap::LaneId& curr_lane_id);
    void PathBoundaryUpdateByObstacles(const zjlmap::LaneId& target_lane_id, const ObstacleCorners& obstacle_corners,
                                       const localPlanner::LaneType& lane_type, const bool& is_self_lane_obstacle,
                                       PathBoundary& path_boundary);

    std::vector<localPlanner::Box2d> obstacle_list_;
    std::shared_ptr<zjlmap::Map> map_data_;
    std::vector<localPlanner::Box2d> static_blocking_obstacle_list;

    double s_buffer = 3.0;
    double l_buffer = 0.5;
    double vehicle_width = 1.0;
    double static_obstacle_lane_change_dis_lowerbound = 20.0;
    double static_obstacle_lane_change_dis_upperbound = 60.0;
    double safe_lane_change_dis_threshold = 5.0;
    double pedestrain_dis_threshold = 5.0;
    double front_close_stop_dis = 3.0;
};

}  // namespace PlanningOptimizer

#pragma once
#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <math.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "header.h"
#include "planning_struct.h"
#include "obstacle_decider.h"

namespace PlanningOptimizer
{
using namespace zjlmap;

class SpeedPlanner
{
   public:
    SpeedPlanner();
    ~SpeedPlanner(){};

    bool Init(const std::shared_ptr<Map>& map_data);
    bool Process(const localPlanner::Box2d& robot, const std::vector<localPlanner::TargetNode> targets,
                 ObstacleDecider& obstacle_decider_handle, std::vector<TrajectoryPoint>& planning_trajectory_data);
    double GetSpeedRefFromCurvature(const std::vector<TrajectoryPoint>& planning_trajectory_data);

    std::shared_ptr<Map> map_data_;
    bool is_need_estop = false;
    double forward_curvature_length = 0.0;
    double max_centric_acc_limit = 0.0;
    double lowest_speed = 0.0;
    double highest_speed = 0.0;
    double pre_desired_speed = 0.0;
    double wheel_base = 1.0;
};

}  // namespace PlanningOptimizer

#endif
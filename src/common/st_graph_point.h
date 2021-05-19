#pragma once

#include <limits>

#include "planning_struct.h"

namespace PlanningOptimizer
{
class StGraphPoint
{
   public:
    StGraphPoint(){};
    ~StGraphPoint(){};

    STPoint st_point;
    const StGraphPoint* pre_point = nullptr;
    std::uint32_t index_s = 0;
    std::uint32_t index_t = 0;

    double optimal_speed = 0.0;
    double obstacle_cost = 0.0;
    double goal_heuristic_cost = 0.0;
    double total_cost = std::numeric_limits<double>::infinity();
};

}  // namespace PlanningOptimizer

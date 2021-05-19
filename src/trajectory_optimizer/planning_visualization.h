#pragma once
#ifndef PLANNING_VISUALIZATION_H
#define PLANNING_VISUALIZATION_H

#include <fstream>
#include <string>

#include "map.h"
#include "st_graph_point.h"
#include "header.h"
#include "planning_struct.h"

namespace PlanningOptimizer
{
class PlanningVisualization
{
   public:
    PlanningVisualization(const std::shared_ptr<zjlmap::Map>& map_data);
    ~PlanningVisualization(){};

    void SLZVectorVisualization(const std::vector<zjlmap::SLZ>& slz_points);
    void PathBoundaryVisualization(const std::vector<std::pair<zjlmap::SLZ, zjlmap::SLZ>>& road_based_sl_boundary);
    void FrenetPathVisualization(const std::vector<double>& optimized_path_l, const std::vector<double>& optimized_path_dl,
                                 const std::vector<double>& optimized_path_ddl);
    void PathFeatureVisualization(const std::vector<CartesianPathPoint>& planning_path_data, std::string file_str);
    void CartesianPathVisualization(const std::vector<CartesianPathPoint>& planning_path_data);
    void PassLaneVisualization(const std::vector<zjlmap::SLZ>& pass_lane);
    void PointVisualization(const localPlanner::Pose& point);
    void STGraphVisualization(const std::vector<std::vector<StGraphPoint>>& cost_table);
    void SpeedDistanceVisualization(const std::vector<double>& distance);
    void SpeedVisualization(const std::vector<double>& velocity);
    void AccelerationVisualization(const std::vector<double>& acceleration);
    void DistanceRefVisualization(const std::vector<double>& ref_s);
    void VehiclePositionVisualization(const localPlanner::Box2d& ego_vehicle);

   private:
    std::shared_ptr<zjlmap::Map> map_data_;
};

}  // namespace PlanningOptimizer

#endif
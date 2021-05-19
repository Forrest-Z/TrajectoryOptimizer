#pragma once

#include <vector>
#include "map.h"

namespace PlanningOptimizer {

class PathBoundary
{
public:
	PathBoundary() {};
	~PathBoundary() {};

	std::vector<std::pair<zjlmap::SLZ, zjlmap::SLZ>> road_based_sl_boundary;
	std::vector<std::pair<double, double>> lane_based_sl_boundary; 
	double start_s = 0.0; 
	
};

} // namespace PlanningOptimizer

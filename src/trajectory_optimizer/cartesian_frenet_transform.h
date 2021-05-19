#pragma once

#include "map.h"
#include <vector>
#include <array>
#include <iostream>
#include <header.h>

#define M_PI       3.14159265358979323846   // pi

namespace PlanningOptimizer {

class CartesianFrenetTransform
{
public:
	CartesianFrenetTransform() = delete;

	static void CartesianToFrenet(const localPlanner::Box2d& ego_vehicle, const zjlmap::TracePoint& target_trace_point,
		std::pair<std::array<double, 3>, std::array<double, 3>>& init_frenet_state);
	static double CalculateTheta(const double rtheta, const double rkappa, const double l, const double dl);
	static double CalculateKappa(const double rkappa, const double rdkappa,
		const double l, const double dl, const double ddl);
};


} // namespace PlanningOptimizer

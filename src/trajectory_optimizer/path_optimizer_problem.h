#pragma once

#include <iostream>
#include <vector>
#include <array>

#include "optimizer_problem.h"

namespace PlanningOptimizer {

class PathOptimizerProblem : public OptimizerProblem
{
public:
	PathOptimizerProblem(const uint32_t num_of_knots, const double delta_s, const std::array<double, 3>& x_init);
	virtual ~PathOptimizerProblem() = default;

	void CreateCostFunc(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr) override;
	void CalculateOffset(std::vector<c_float>* q) override;
	void CalculateConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
		std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) override;
};

} // namespace PlanningOptimizer

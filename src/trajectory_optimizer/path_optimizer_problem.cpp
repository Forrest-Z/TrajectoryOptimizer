#include "path_optimizer_problem.h"

namespace PlanningOptimizer {

const int MAX_VARIABLE_RANGE = INT_MAX;

PathOptimizerProblem::PathOptimizerProblem(const uint32_t num_of_knots, const double delta_s, const std::array<double, 3>& x_init)
	: OptimizerProblem(num_of_knots, delta_s, x_init) {}

void PathOptimizerProblem::CreateCostFunc(std::vector<c_float>* P_data,
	std::vector<c_int>* P_indices,
	std::vector<c_int>* P_indptr)
{
	const int n = static_cast<int>(num_of_knots_);
	const int num_of_variables = 3 * n;
	// const int num_of_nonzeros = num_of_variables + (n - 1);
	std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
	int value_index = 0;

	// x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i)
	// or piecewise values for different x(i)
	for (int i = 0; i < n - 1; ++i) {
		columns[i].emplace_back(i, weight_x_ /
			(scale_factor_[0] * scale_factor_[0]));
		++value_index;
	}
	// x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x)
	columns[n - 1].emplace_back(
		n - 1, (weight_x_ + weight_end_state_[0]) /
		(scale_factor_[0] * scale_factor_[0]));
	++value_index;

	// x(i)'^2 * w_dx
	for (int i = 0; i < n - 1; ++i) {
		columns[n + i].emplace_back(
			n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
		++value_index;
	}
	// x(n-1)'^2 * (w_dx + w_end_dx)
	columns[2 * n - 1].emplace_back(2 * n - 1,
		(weight_dx_ + weight_end_state_[1]) /
		(scale_factor_[1] * scale_factor_[1]));
	++value_index;

	auto delta_s_square = delta_s_ * delta_s_;
	// x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
	columns[2 * n].emplace_back(2 * n,
		(weight_ddx_ + weight_dddx_ / delta_s_square) /
		(scale_factor_[2] * scale_factor_[2]));
	++value_index;
	for (int i = 1; i < n - 1; ++i) {
		columns[2 * n + i].emplace_back(
			2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
			(scale_factor_[2] * scale_factor_[2]));
		++value_index;
	}
	columns[3 * n - 1].emplace_back(
		3 * n - 1,
		(weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
		(scale_factor_[2] * scale_factor_[2]));
	++value_index;

	// -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
	for (int i = 0; i < n - 1; ++i) {
		columns[2 * n + i].emplace_back(2 * n + i + 1,
			(-2.0 * weight_dddx_ / delta_s_square) /
			(scale_factor_[2] * scale_factor_[2]));
		++value_index;
	}

	int ind_p = 0;
	for (int i = 0; i < num_of_variables; ++i) {
		P_indptr->push_back(ind_p);
		for (const auto& row_data_pair : columns[i]) {
			P_data->push_back(row_data_pair.second * 2.0);
			P_indices->push_back(row_data_pair.first);
			++ind_p;
		}
	}
	P_indptr->push_back(ind_p);
}


void PathOptimizerProblem::CalculateOffset(std::vector<c_float>* q) {
	const int n = static_cast<int>(num_of_knots_);
	const int kNumParam = 3 * n;
	q->resize(kNumParam, 0.0);

}

void PathOptimizerProblem::CalculateConstraint(
	std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
	std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
	std::vector<c_float>* upper_bounds) {
	// 3N params bounds on x, x', x''
	// 3(N-1) constraints on x, x', x''
	// 3 constraints on x_init_
	const int n = static_cast<int>(num_of_knots_);
	const int num_of_variables = 3 * n;
	size_t num_of_constraints = 0;
	num_of_constraints = num_of_variables + 3 * (n - 1) + 3 + 1;

	lower_bounds->resize(num_of_constraints);
	upper_bounds->resize(num_of_constraints);

	std::vector<std::vector<std::pair<c_int, c_float>>> variables(
		num_of_variables);

	int constraint_index = 0;
	// set x, x', x'' bounds
	for (int i = 0; i < num_of_variables; ++i) {
		if (i < n) {
			variables[i].emplace_back(constraint_index, 1.0);
			lower_bounds->at(constraint_index) =
				x_bounds_[i].first * scale_factor_[0];
			upper_bounds->at(constraint_index) =
				x_bounds_[i].second * scale_factor_[0];
		}
		else if (i < 2 * n) {
			variables[i].emplace_back(constraint_index, 1.0);

			lower_bounds->at(constraint_index) =
				dx_bounds_[i - n].first * scale_factor_[1];
			upper_bounds->at(constraint_index) =
				dx_bounds_[i - n].second * scale_factor_[1];
		}
		else {
			variables[i].emplace_back(constraint_index, 1.0);
			lower_bounds->at(constraint_index) =
				ddx_bounds_[i - 2 * n].first * scale_factor_[2];
			upper_bounds->at(constraint_index) =
				ddx_bounds_[i - 2 * n].second * scale_factor_[2];
		}
		++constraint_index;
	}

	// x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
	for (int i = 0; i + 1 < n; ++i) {
		variables[2 * n + i].emplace_back(constraint_index, -1.0);
		variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
		lower_bounds->at(constraint_index) =
			dddx_bound_.first * delta_s_ * scale_factor_[2];
		upper_bounds->at(constraint_index) =
			dddx_bound_.second * delta_s_ * scale_factor_[2];
		++constraint_index;
	}

	// x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
	for (int i = 0; i + 1 < n; ++i) {
		variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
		variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
		variables[2 * n + i].emplace_back(constraint_index,
			-0.5 * delta_s_ * scale_factor_[1]);
		variables[2 * n + i + 1].emplace_back(constraint_index,
			-0.5 * delta_s_ * scale_factor_[1]);
		lower_bounds->at(constraint_index) = 0.0;
		upper_bounds->at(constraint_index) = 0.0;
		++constraint_index;
	}

	// x(i+1) - x(i) - delta_s * x(i)'
	// - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
	auto delta_s_sq_ = delta_s_ * delta_s_;
	for (int i = 0; i + 1 < n; ++i) {
		variables[i].emplace_back(constraint_index,
			-1.0 * scale_factor_[1] * scale_factor_[2]);
		variables[i + 1].emplace_back(constraint_index,
			1.0 * scale_factor_[1] * scale_factor_[2]);
		variables[n + i].emplace_back(
			constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
		variables[2 * n + i].emplace_back(
			constraint_index,
			-delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
		variables[2 * n + i + 1].emplace_back(
			constraint_index,
			-delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

		lower_bounds->at(constraint_index) = 0.0;
		upper_bounds->at(constraint_index) = 0.0;
		++constraint_index;
	}

	// constrain on x_init
	variables[0].emplace_back(constraint_index, 1.0);
	lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
	upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
	++constraint_index;

	variables[n].emplace_back(constraint_index, 1.0);
	lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
	upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
	++constraint_index;

	variables[2 * n].emplace_back(constraint_index, 1.0);
	lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
	upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
	++constraint_index;

	// add constrain on x_end for path optimizer
	variables[n - 1].emplace_back(constraint_index, 1.0);
	lower_bounds->at(constraint_index) = 0.0;
	upper_bounds->at(constraint_index) = 0.0;

	int ind_p = 0;
	for (int i = 0; i < num_of_variables; ++i) {
		A_indptr->push_back(ind_p);
		for (const auto& variable_nz : variables[i]) {
			// coefficient
			A_data->push_back(variable_nz.second);

			// constraint index
			A_indices->push_back(variable_nz.first);
			++ind_p;
		}
	}

	A_indptr->push_back(ind_p);
}


} // namespace PlanningOptimizer
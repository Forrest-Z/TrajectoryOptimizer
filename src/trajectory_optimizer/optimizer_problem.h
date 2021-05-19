#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <limits.h>
#include "osqp/osqp.h"

namespace PlanningOptimizer {

	class OptimizerProblem
	{
	public:
		OptimizerProblem(const uint32_t num_of_knots, const double delta_s, const std::array<double, 3>& x_init);
		virtual ~OptimizerProblem() = default;

		bool Optimize(const int max_iter);
		OSQPData* FormulateProblem();
		void FreeData(OSQPData* data);


		virtual void CreateCostFunc(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr) = 0;
		virtual void CalculateOffset(std::vector<c_float>* q) = 0;
		virtual void CalculateConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
			std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) = 0;
		

		template <typename T>
		T* CopyData(const std::vector<T>& vec) {
			T* data = new T[vec.size()];
			memcpy(data, vec.data(), sizeof(T) * vec.size());
			return data;
		}

		uint32_t num_of_knots_ = 0;
		std::array<double, 3> x_init_;
		std::array<double, 3> scale_factor_ = { {1.0, 1.0, 1.0} };
		double delta_s_ = 1.0;

		std::vector<double> x_;
		std::vector<double> dx_;
		std::vector<double> ddx_;

		std::vector<std::pair<double, double>> x_bounds_;
		std::vector<std::pair<double, double>> dx_bounds_;
		std::vector<std::pair<double, double>> ddx_bounds_;
		std::pair<double, double> dddx_bound_;

		double weight_x_ = 0.0;
		double weight_dx_ = 0.0;
		double weight_ddx_ = 0.0;
		double weight_dddx_ = 0.0;
		std::array<double, 3> weight_end_state_ = { {0.0, 0.0, 0.0} };

		bool has_x_ref_ = false;
		double weight_x_ref_ = 0.0;
		std::vector<double> x_ref_;

		bool has_end_state_ref_ = false;
		std::array<double, 3> end_state_ref_;

	};

} // namespace PlanningOptimizer

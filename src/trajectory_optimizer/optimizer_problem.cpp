#include "optimizer_problem.h"

namespace PlanningOptimizer {

	const int MAX_VARIABLE_RANGE = INT_MAX;

	OptimizerProblem::OptimizerProblem(const uint32_t num_of_knots, const double delta_s, const std::array<double, 3>& x_init)
	{
		num_of_knots_ = num_of_knots;
		x_init_ = x_init;
		delta_s_ = delta_s;

		x_bounds_.resize(num_of_knots_,
			std::make_pair(-MAX_VARIABLE_RANGE, MAX_VARIABLE_RANGE));

		dx_bounds_.resize(num_of_knots_,
			std::make_pair(-MAX_VARIABLE_RANGE, MAX_VARIABLE_RANGE));

		ddx_bounds_.resize(num_of_knots_,
			std::make_pair(-MAX_VARIABLE_RANGE, MAX_VARIABLE_RANGE));
		// weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
	}


	OSQPData* OptimizerProblem::FormulateProblem() {
		// calculate kernel
		std::vector<c_float> P_data;
		std::vector<c_int> P_indices;
		std::vector<c_int> P_indptr;
		CreateCostFunc(&P_data, &P_indices, &P_indptr);

		// calculate affine constraints
		std::vector<c_float> A_data;
		std::vector<c_int> A_indices;
		std::vector<c_int> A_indptr;
		std::vector<c_float> lower_bounds;
		std::vector<c_float> upper_bounds;
		CalculateConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
			&upper_bounds);

		// calculate offset
		std::vector<c_float> q;
		CalculateOffset(&q);

		OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

		size_t kernel_dim = 3 * num_of_knots_;
		size_t num_affine_constraint = lower_bounds.size();

		data->n = kernel_dim;
		data->m = num_affine_constraint;
		data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
			CopyData(P_indices), CopyData(P_indptr));
		data->q = CopyData(q);
		data->A =
			csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
				CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
		data->l = CopyData(lower_bounds);
		data->u = CopyData(upper_bounds);
		return data;
	}

	bool OptimizerProblem::Optimize(const int max_iter)
	{
		OSQPData* data = FormulateProblem();
		std::cout << "Formulate Problem is finished! " << std::endl; 

		OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
		osqp_set_default_settings(settings);
		settings->polish = true;
		settings->verbose = true;
		settings->scaled_termination = true;
		std::cout << "osqp setting is finished! " << std::endl; 

		settings->max_iter = max_iter;

		OSQPWorkspace* osqp_work = nullptr;
		osqp_work = osqp_setup(data, settings);
		// osqp_setup(&osqp_work, data, settings);
		std::cout << "osqp setup is finished! " << std::endl; 

		osqp_solve(osqp_work);
		std::cout << "osqp solve is finished! " << std::endl; 

		auto status = osqp_work->info->status_val;
		std::cout << "osqp status  is finished! " << std::endl; 

		if (status < 0 || (status != 1 && status != 2)) {
			std::cout << "optimzier failed optimization status: " << osqp_work->info->status << std::endl;
			osqp_cleanup(osqp_work);
			FreeData(data);
			c_free(settings);
			return false;
		}
		else if (osqp_work->solution == nullptr) {
			std::cout << "The solution from OSQP is nullptr! " << std::endl;
			osqp_cleanup(osqp_work);
			FreeData(data);
			c_free(settings);
			return false;
		}

		// debug the osqp result  
		std::cout << "OSQP Solver status: " << osqp_work->info->status << std::endl;
		std::cout << "OSQP Solver num of iterations: " << osqp_work->info->iter << std::endl;


		// extract primal results
		x_.resize(num_of_knots_);
		dx_.resize(num_of_knots_);
		ddx_.resize(num_of_knots_);
		for (size_t i = 0; i < num_of_knots_; ++i) {
			x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
			dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
			ddx_.at(i) =
				osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
		}

		// Cleanup
		osqp_cleanup(osqp_work);
		FreeData(data);
		c_free(settings);
		return true;
	}

	
	


	void OptimizerProblem::FreeData(OSQPData* data) {
		delete[] data->q;
		delete[] data->l;
		delete[] data->u;

		delete[] data->P->i;
		delete[] data->P->p;
		delete[] data->P->x;

		delete[] data->A->i;
		delete[] data->A->p;
		delete[] data->A->x;
	}

} // namespace PlanningOptimizer
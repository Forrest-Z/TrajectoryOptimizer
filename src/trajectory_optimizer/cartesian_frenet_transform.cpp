#include "cartesian_frenet_transform.h"

namespace PlanningOptimizer {

double NormalizeAngle(const double angle)
{
	double a = std::fmod(angle + M_PI, 2.0 * M_PI);
	if (a < 0.0)
	{
		a += (2.0 * M_PI);
	}
	return a - M_PI;
}

void CartesianFrenetTransform::CartesianToFrenet(const localPlanner::Box2d& ego_vehicle, const zjlmap::TracePoint& target_trace_point, 
	std::pair<std::array<double, 3>, std::array<double, 3>>& init_frenet_state)
{
	const double dx = ego_vehicle.x - target_trace_point.x;
	const double dy = ego_vehicle.y - target_trace_point.y;

	const double cos_theta_r = std::cos(target_trace_point.hdg);
	const double sin_theta_r = std::sin(target_trace_point.hdg);

	const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
	// init_frenet_state.second[0] = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
	int sign = abs(ego_vehicle.rlsl.l) > abs(target_trace_point.l) ? 1 : -1;
	init_frenet_state.second[0] = sign * (abs(ego_vehicle.rlsl.l - target_trace_point.l));

	// const double delta_theta = planning_start_point.path_point.theta - target_trace_point.hdg;
	const double delta_theta = target_trace_point.hdg - ego_vehicle.theta; 
	std::cout << "[debug] target_trace_point.hdg: " << target_trace_point.hdg << std::endl;
	const double tan_delta_theta = std::tan(delta_theta);
	const double cos_delta_theta = std::cos(delta_theta);

	const double one_minus_kappa_r_d = 1 - target_trace_point.curv * init_frenet_state.second[0];
	init_frenet_state.second[1] = one_minus_kappa_r_d * tan_delta_theta;

	const double kappa_r_d_prime =
		target_trace_point.curv_deriv * init_frenet_state.second[0] + target_trace_point.curv * init_frenet_state.second[1];


	std::cout << "[Debug] ddl calculation kappa_r_d_prime: " << kappa_r_d_prime << " tan_delta_theta: " << tan_delta_theta << " one_minus_kappa_r_d: " <<
		one_minus_kappa_r_d << " cos_delta_theta: " << cos_delta_theta << " planning_start_point.path_point.kappa: " << ego_vehicle.kappa <<
		" target_trace_point.curv: " << target_trace_point.curv << std::endl; 
	init_frenet_state.second[2] =
		-kappa_r_d_prime * tan_delta_theta +
		one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
		(ego_vehicle.kappa * one_minus_kappa_r_d / cos_delta_theta - target_trace_point.curv);

	init_frenet_state.first[0] = target_trace_point.s;

	init_frenet_state.first[1] = ego_vehicle.speed * cos_delta_theta / one_minus_kappa_r_d;

	const double delta_theta_prime =
		one_minus_kappa_r_d / cos_delta_theta * ego_vehicle.kappa - target_trace_point.curv;
	init_frenet_state.first[2] =
		(ego_vehicle.acc * cos_delta_theta -
			init_frenet_state.first[1] * init_frenet_state.first[1] *
			(init_frenet_state.second[1] * delta_theta_prime - kappa_r_d_prime)) /
		one_minus_kappa_r_d;
}

double CartesianFrenetTransform::CalculateTheta(const double rtheta, const double rkappa, const double l, const double dl) 
{
	return NormalizeAngle(rtheta + std::atan2(dl, 1 - l * rkappa));
}

double CartesianFrenetTransform::CalculateKappa(const double rkappa, const double rdkappa, 
	const double l, const double dl, const double ddl) 
{
	double denominator = (dl * dl + (1 - l * rkappa) * (1 - l * rkappa));
	if (std::fabs(denominator) < 1e-8) 
	{
		return 0.0;
	}
	denominator = std::pow(denominator, 1.5);
	const double numerator = rkappa + ddl - 2 * l * rkappa * rkappa -
		l * ddl * rkappa + l * l * rkappa * rkappa * rkappa +
		l * dl * rdkappa + 2 * dl * dl * rkappa;
	return numerator / denominator;
}

} // namespace PlanningOptimizer
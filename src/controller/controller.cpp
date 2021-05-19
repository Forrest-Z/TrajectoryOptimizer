#include "controller.h"

TrajectoryPoint Controller::FindMinDistancePoint(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data)
{
	double min_dis = 999999.0;
	for (size_t i = 0; i < planning_trajectory_data.size(); i++)
	{
		double distance = std::sqrt(pow((planning_trajectory_data[i].path_point.x - ego_vehicle.x), 2)+
			pow((planning_trajectory_data[i].path_point.y - ego_vehicle.y), 2));
		if (distance < min_dis)
		{
			min_dis = distance; 
			nearest_point_index = i;
		}
	}

	return planning_trajectory_data[nearest_point_index];
}

TrajectoryPoint Controller::FindTargetControlPoint(const std::vector<TrajectoryPoint>& planning_trajectory_data, const TrajectoryPoint& nearest_point)
{
	double accumulate_s = 0.0; 
	for (int i = nearest_point_index; i < planning_trajectory_data.size()-1; i++)
	{
		accumulate_s += std::sqrt(pow((planning_trajectory_data[i].path_point.x - planning_trajectory_data[i+1].path_point.x), 2) +
			pow((planning_trajectory_data[i].path_point.y - planning_trajectory_data[i+1].path_point.y), 2));
		if (accumulate_s >= forward_length)
		{
			return planning_trajectory_data[i + 1];
		}
	}

	return planning_trajectory_data.back();
}

void Controller::LateralControl(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data, double& steering_wheel_angle)
{
	forward_length = forward_time * ego_vehicle.speed / 3.6;
	TrajectoryPoint nearest_point = FindMinDistancePoint(ego_vehicle, planning_trajectory_data);

	TrajectoryPoint target_control_point;
	if (nearest_point_index == planning_trajectory_data.size())
	{
		target_control_point = planning_trajectory_data.back();
	}
	else
	{
		target_control_point = FindTargetControlPoint(planning_trajectory_data, nearest_point);
	}

	// steering_wheel_angle = std::atan(planning_trajectory_data[0].path_point.kappa * wheel_base) / max_steering_wheel_angle * 100;
	steering_wheel_angle = std::atan(planning_trajectory_data[1].path_point.kappa * wheel_base) / max_steering_wheel_angle * 100;
	// steering_wheel_angle = std::atan(target_control_point.path_point.kappa * wheel_base) / max_steering_wheel_angle * 100;
	// std::cout << "[Debug control] target control point kappa: " << target_control_point.path_point.kappa << std::endl;
	std::cout << "[Debug control] planning_trajectory_data[1].path_point.kappa: " << planning_trajectory_data[1].path_point.kappa << std::endl;


	steering_wheel_angle = std::max(steering_wheel_angle, -100.0);
	steering_wheel_angle = std::min(steering_wheel_angle, 100.0);

	return;
}

void Controller::LongitudinalControl(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data, double& cmd)
{
	double speed_error = planning_trajectory_data[0].v - ego_vehicle.speed;
	std::cout << "[Debug] desired_speed: " << planning_trajectory_data[0].v << " actual speed: " << ego_vehicle.speed << std::endl;
	std::cout << "[Debug] speed_error: " << speed_error << std::endl; 
	double speed_controller_input_limit = 2.0;
	double speed_controller_input_limited = std::min(std::max(speed_error, -speed_controller_input_limit), speed_controller_input_limit);

	PidConf pid_conf;
	pid_conf.integrator_enable = true;
	pid_conf.integrator_saturation_level = 0.3;
	pid_conf.kp = 30.0;
	pid_conf.ki = 0.3;
	pid_conf.kd = 0.0;
	speed_pid_controller.Init(pid_conf);

	cmd = speed_pid_controller.Control(speed_error, 0.1);
	std::cout << "[Debug] Longitudinal control cmd: " << cmd << std::endl; 

	return;
}
#pragma once

#include <vector>
#include <math.h>
#include <fstream>
#include <header.h>
#include "planning_struct.h"
#include "pid_controller.h"

#define M_PI       3.14159265358979323846   // pi

using namespace PlanningOptimizer;

class Controller
{
public:
	Controller() {};
	~Controller() {};
	void LateralControl(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data, double& steering_wheel_angle);
	void LongitudinalControl(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data, double& cmd);
	TrajectoryPoint FindMinDistancePoint(const localPlanner::Box2d& ego_vehicle, const std::vector<TrajectoryPoint>& planning_trajectory_data);
	TrajectoryPoint FindTargetControlPoint(const std::vector<TrajectoryPoint>& planning_trajectory_data, const TrajectoryPoint& nearest_point);

private:
	PIDController speed_pid_controller;

	int index = 0;
	double forward_time = 1.5;
	double forward_length = 0.0;
	int nearest_point_index = 0;
	double wheel_base = 1.0;
	double max_steering_wheel_angle = 0.6981;
};


#pragma once

#include <vector>
#include "map.h"

namespace PlanningOptimizer {

// road based frenet frame path point 
struct FrenetFramePathPoint
{
	zjlmap::LaneId lane_id;
	double s;
	double l;
	double z;
	double dl;
	double ddl; 
};

struct CartesianPathPoint
{
	zjlmap::LaneId lane_id;
	double s;
	double x;
	double y;
	double z;
	double theta;
	double kappa;
	double accumulate_dis;
};

struct TrajectoryPoint
{
	CartesianPathPoint path_point;
	double v;
	double a;
	double da;
    double relative_time;
};

struct PathBoundary
{
    std::vector<std::pair<zjlmap::SLZ, zjlmap::SLZ>> road_based_sl_boundary;
    std::vector<std::pair<double, double>> lane_based_sl_boundary;
    double start_s;
};

struct NavigationData
{
	zjlmap::Route routing_result;
	bool is_rerouting;
};

enum RoleType 
{ 
	VEHICLE = 0,
	PEDESTRIAN = 1, 
	UNKNOWN = 2
};

struct Box2d
{
    std::string id;

    RoleType roleType;  //  -1:others; 0�� �������� 1�����ˣ� 2���ǻ������� 3����� 4�� ���� 5�� �źŵ�.

    zjlmap::SLZ slz;

    bool is_static;
    bool is_virtual;

    double x;
    double y;
    double z;
    double distance;
    double theta;
    double speed;
    double acc;

    double width;
    double length;
};

struct STPoint
{
    double s;
    double t;
    double v;
    double a;
    double da;
};

struct PlanningInfo
{
	std::vector<CartesianPathPoint> planning_path_data;

};

} // namespace PlanningOptimizer

#include "planning_visualization.h"

namespace PlanningOptimizer {

PlanningVisualization::PlanningVisualization(const std::shared_ptr<zjlmap::Map>& map_data)
{
	map_data_ = map_data;
}

void PlanningVisualization::SLZVectorVisualization(const std::vector<zjlmap::SLZ>& slz_points)
{
	std::vector<zjlmap::XYZ> utm_target_line_points;
    std::ofstream target_line_cout("/apollo/modules/data/planning_visualization_data/target_line.txt");
    target_line_cout.setf(std::ios::fixed, std::ios::floatfield);
	target_line_cout.precision(2);
	for (size_t i = 0; i < slz_points.size(); i++)
	{
		zjlmap::XYZ xyz_point = map_data_->xyz(slz_points[i]);
		target_line_cout << xyz_point.x << "  " << xyz_point.y << std::endl;
		utm_target_line_points.push_back(xyz_point);
	}
	target_line_cout.close();
}

void PlanningVisualization::PathBoundaryVisualization(const std::vector<std::pair<zjlmap::SLZ, zjlmap::SLZ>>& road_based_sl_boundary)
{
	std::vector<zjlmap::XYZ> utm_left_path_boundary;
	std::vector<zjlmap::XYZ> utm_right_path_boundary;
    std::ofstream left_boundary_cout("/apollo/modules/data/planning_visualization_data/left_boundary.txt");
    left_boundary_cout.setf(std::ios::fixed, std::ios::floatfield);
	left_boundary_cout.precision(2);
	for (size_t i = 0; i < road_based_sl_boundary.size(); i++)
	{
		zjlmap::XYZ left_boundary_point_xyz = map_data_->xyz(road_based_sl_boundary[i].first);
		left_boundary_cout << left_boundary_point_xyz.x << "  " << left_boundary_point_xyz.y << std::endl;
		utm_left_path_boundary.push_back(left_boundary_point_xyz);
	}
	left_boundary_cout.close();

    std::ofstream right_boundary_cout("/apollo/modules/data/planning_visualization_data/right_boundary.txt");
    right_boundary_cout.setf(std::ios::fixed, std::ios::floatfield);
	right_boundary_cout.precision(2);
	for (size_t i = 0; i < road_based_sl_boundary.size(); i++)
	{
		zjlmap::XYZ right_boundary_point_xyz = map_data_->xyz(road_based_sl_boundary[i].second);
		right_boundary_cout << right_boundary_point_xyz.x << "  " << right_boundary_point_xyz.y << std::endl;
		utm_right_path_boundary.push_back(right_boundary_point_xyz);
	}
	right_boundary_cout.close();
}

void PlanningVisualization::FrenetPathVisualization(const std::vector<double>& optimized_path_l, const std::vector<double>& optimized_path_dl, 
	const std::vector<double>& optimized_path_ddl)
{
    std::ofstream optimized_sl_path_cout("/apollo/modules/data/planning_visualization_data/optimized_sl_path.txt");
    optimized_sl_path_cout.setf(std::ios::fixed, std::ios::floatfield);
	optimized_sl_path_cout.precision(7);
	for (size_t i = 0; i < optimized_path_l.size(); i++)
	{
		optimized_sl_path_cout << i << " " << optimized_path_l[i] << "  " << optimized_path_dl[i] << " " << optimized_path_ddl[i] << std::endl;
	}
	optimized_sl_path_cout.close();
}

void PlanningVisualization::PathFeatureVisualization(const std::vector<CartesianPathPoint>& planning_path_data, std::string file_str)
{
    std::ofstream path_feature_cout("/apollo/modules/data/planning_visualization_data/" + file_str + ".txt");
    path_feature_cout.setf(std::ios::fixed, std::ios::floatfield);
	path_feature_cout.precision(4);
	
	if (file_str == "theta")
	{
		for (size_t i = 0; i < planning_path_data.size(); i++)
		{
			path_feature_cout << i << " " << planning_path_data[i].theta << std::endl;
		}
	}
	else if (file_str == "kappa")
	{
		for (size_t i = 0; i < planning_path_data.size(); i++)
		{
			path_feature_cout << i << " " << planning_path_data[i].kappa << std::endl;
		}
	}
	
	path_feature_cout.close();
}

void PlanningVisualization::CartesianPathVisualization(const std::vector<CartesianPathPoint>& planning_path_data)
{
    std::ofstream optimized_path_cout("/apollo/modules/data/planning_visualization_data/optimized_path.txt");
    optimized_path_cout.setf(std::ios::fixed, std::ios::floatfield);
	optimized_path_cout.precision(2);
	for (size_t i = 0; i < planning_path_data.size(); i++)
	{
		optimized_path_cout << planning_path_data[i].x << "  " << planning_path_data[i].y << std::endl;
	}
	optimized_path_cout.close();
}

void PlanningVisualization::PassLaneVisualization(const std::vector<zjlmap::SLZ>& pass_lane)
{
	std::vector<zjlmap::XYZ> utm_pass_lane;
    std::ofstream utm_pass_lane_cout("/apollo/modules/data/planning_visualization_data/pass_lane.txt");
    utm_pass_lane_cout.setf(std::ios::fixed, std::ios::floatfield);
	utm_pass_lane_cout.precision(2);
	for (size_t i = 0; i < pass_lane.size(); i++)
	{
		zjlmap::XYZ pass_lane_point_xyz = map_data_->xyz(pass_lane[i]);
		utm_pass_lane_cout << pass_lane_point_xyz.x << "  " << pass_lane_point_xyz.y << std::endl;
	}
	utm_pass_lane_cout.close();
}

void PlanningVisualization::PointVisualization(const localPlanner::Pose& point)
{
    std::ofstream target_point_cout("/apollo/modules/data/planning_visualization_data/target_point.txt");
    target_point_cout.setf(std::ios::fixed, std::ios::floatfield);
	target_point_cout.precision(5);
	target_point_cout << point.x << " " << point.y << std::endl; 
	target_point_cout.close();
}

void PlanningVisualization::STGraphVisualization(const std::vector<std::vector<StGraphPoint>>& cost_table)
{
    std::ofstream st_graph_cout("/apollo/modules/data/planning_visualization_data/st_cost_table.txt");
    st_graph_cout.setf(std::ios::fixed, std::ios::floatfield);
	st_graph_cout.precision(3);
	for (int i = cost_table[0].size() - 1; i >= 0; i--)
	{
		for (size_t j = 0; j < cost_table.size(); j++)
		{
			st_graph_cout << cost_table[j][i].total_cost << " ";
		}
		st_graph_cout << std::endl;

	}
	st_graph_cout.close();
}

void PlanningVisualization::SpeedDistanceVisualization(const std::vector<double>& distance)
{
    std::ofstream speed_distance_cout("/apollo/modules/data/planning_visualization_data/st_distance.txt");
    speed_distance_cout.setf(std::ios::fixed, std::ios::floatfield);
	speed_distance_cout.precision(2);
	for (size_t i = 0; i < distance.size(); i++)
	{
		speed_distance_cout << i << "  " << distance[i] << std::endl;
	}
	speed_distance_cout.close();
}

void PlanningVisualization::SpeedVisualization(const std::vector<double>& velocity)
{
    std::ofstream speed_velocity_cout("/apollo/modules/data/planning_visualization_data/st_velocity.txt");
    speed_velocity_cout.setf(std::ios::fixed, std::ios::floatfield);
	speed_velocity_cout.precision(2);
	for (size_t i = 0; i < velocity.size(); i++)
	{
		speed_velocity_cout << i << "  " << velocity[i] << std::endl;
	}
	speed_velocity_cout.close();
}

void PlanningVisualization::AccelerationVisualization(const std::vector<double>& acceleration)
{
    std::ofstream speed_acc_cout("/apollo/modules/data/planning_visualization_data/st_acceleration.txt");
    speed_acc_cout.setf(std::ios::fixed, std::ios::floatfield);
	speed_acc_cout.precision(2);
	for (size_t i = 0; i < acceleration.size(); i++)
	{
		speed_acc_cout << i << "  " << acceleration[i] << std::endl;
	}
	speed_acc_cout.close();
}

void PlanningVisualization::DistanceRefVisualization(const std::vector<double>& ref_s)
{
    std::ofstream distance_ref_cout("/apollo/modules/data/planning_visualization_data/distance_ref.txt");
    distance_ref_cout.setf(std::ios::fixed, std::ios::floatfield);
	distance_ref_cout.precision(2);
	for (size_t i = 0; i < ref_s.size(); i++)
	{
		distance_ref_cout << i << "  " << ref_s[i] << std::endl;
	}
	distance_ref_cout.close();
}

void PlanningVisualization::VehiclePositionVisualization(const localPlanner::Box2d& ego_vehicle)
{
	std::ofstream append_vehicle_pos_fout("apollo/modules/data/planning_visualization_data/vehicle_pos.txt",
                                               std::ofstream::app);
	append_vehicle_pos_fout.setf(std::ios::fixed, std::ios::floatfield);
	append_vehicle_pos_fout.precision(2);
	append_vehicle_pos_fout << ego_vehicle.x << " " << ego_vehicle.y << std::endl;
	append_vehicle_pos_fout.close();
}

} // namespace PlanningOptimizer
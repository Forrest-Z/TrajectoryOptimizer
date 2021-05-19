#include <iostream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <ctime>
#include <sys/timeb.h>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include "map.h"
#include "LibDrive.h" // SDK 接口.
#include "adapter_gflags.h"
#include "trajectory_planner.h"
#include "controller.h"
#include "local_target_generator.h"
#include "log_manager.h"

#include "chassis_messages.pb.h" //底盘protobuf头文件.
#include "chassis_enums.pb.h" //底盘protobuf头文件.
#include "messages.pb.h"  // pubrole protobuf头文件.


using namespace zjlmap;

vts::protocol::main::Notify protoNotify;
vts::protocol::main::PubRole protoPubRole;
vts::protocol::chassis::VehicleFeedback protoVehicleFB;

std::thread thread_;
std::mutex m;

bool start_test = 0;
std::string role_id;
std::vector<localPlanner::Box2d> obstacles;
localPlanner::Box2d robot;
bool is_robot_init = false;
bool is_obstacle_init = false;
const double latency = 0.1; // 100 ms
const double Lf = 1.0f;


// Get yaw from quaternion.
double GetYawfromQuaternion(double x, double y, double z, double w)
{
	double siny_cosp = 2 * (w * z + x * y);
	double cosy_cosp = 1 - 2 * (y * y + z * z);
	return std::atan2(siny_cosp, cosy_cosp);
}

// 结束程序时清空Notify数据.
void ReleaseResMsg()
{
	for (size_t i = 0; i < 10; i++)
	{
		/* code */
		vts::drive::VTS_GetNotify(protoNotify);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

// 退出函数.
void main_exit()
{
	char enters[12] = { '\0' };
	while (strcmp(enters, "quit") != 0)
	{
		printf("enter 'quit' to end \n");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// scanf_s( "%s", enters, 12 );
	}
	ReleaseResMsg();// 结束程序时清空Notify数据.
	printf("main has quitted\n");
}

void GetNewPubRole()
{
	std::lock_guard<std::mutex> g1(m);
	while (start_test)
	{
		unsigned int remaining_number_pubrole = 0;
		int ret = vts::drive::VTS_GetPubRole(protoPubRole, vts::drive::GetMessageMode::kLastMessage, remaining_number_pubrole);//接收PubRole信息

		if (ret >= 0)
		{
			if (protoPubRole.s_roles().size() == 1 && protoPubRole.s_roles()[0].id() == role_id)
			{
				robot.id = protoPubRole.s_roles()[0].id();
				robot.width = protoPubRole.s_roles()[0].box().size().y();
				robot.length = protoPubRole.s_roles()[0].box().size().x();
				robot.x = protoPubRole.s_roles()[0].box().bottom_center().x();
				robot.y = protoPubRole.s_roles()[0].box().bottom_center().y();
				robot.z = protoPubRole.s_roles()[0].box().bottom_center().z();

				double vx, vy, vz;
				vx = protoPubRole.s_roles()[0].linear_speed().x();
				vy = protoPubRole.s_roles()[0].linear_speed().y();
				vz = protoPubRole.s_roles()[0].linear_speed().z();
				robot.speed = std::sqrt(vx * vx + vy * vy + vz * vz);

				double ax, ay, az;
				ax = protoPubRole.s_roles()[0].linear_acceleration().x();
				ay = protoPubRole.s_roles()[0].linear_acceleration().y();
				az = protoPubRole.s_roles()[0].linear_acceleration().z();
				robot.acc = std::sqrt(ax * ax + ay * ay + az * az);

				double qx, qy, qz, qw;
				qx = protoPubRole.s_roles()[0].box().rotation().x();
				qy = protoPubRole.s_roles()[0].box().rotation().y();
				qz = protoPubRole.s_roles()[0].box().rotation().z();
				qw = protoPubRole.s_roles()[0].box().rotation().w();
				robot.theta = GetYawfromQuaternion(qx, qy, qz, qw);

				robot.is_static = false;
				robot.is_virtual = false;
				if (!is_robot_init)
				{
					std::cout << "The robot pose is init. " << std::endl;
					is_robot_init = true;
				}
			}
			else
			{
				std::cout << "obstacles num: " << protoPubRole.s_roles().size() << std::endl;
				obstacles.clear();
				for (int i = 0; i < protoPubRole.s_roles().size(); i++)
				{
					localPlanner::Box2d obstacle;
					obstacle.id = protoPubRole.s_roles()[i].id();
					obstacle.width = protoPubRole.s_roles()[i].box().size().y();
					obstacle.length = protoPubRole.s_roles()[i].box().size().x();
					obstacle.x = protoPubRole.s_roles()[i].box().bottom_center().x();
					obstacle.y = protoPubRole.s_roles()[i].box().bottom_center().y();
					obstacle.z = protoPubRole.s_roles()[i].box().bottom_center().z();

					double dist2robot = sqrt((obstacle.x - robot.x)*(obstacle.x - robot.x) + (obstacle.y - robot.y)*(obstacle.y - robot.y));
					if (dist2robot > 50.0)
					{
						continue;
					}

					double vx, vy, vz;
					vx = protoPubRole.s_roles()[i].linear_speed().x();
					vy = protoPubRole.s_roles()[i].linear_speed().y();
					vz = protoPubRole.s_roles()[i].linear_speed().z();
					obstacle.speed = std::sqrt(vx * vx + vy * vy + vz * vz);

					double ax, ay, az;
					ax = protoPubRole.s_roles()[i].linear_acceleration().x();
					ay = protoPubRole.s_roles()[i].linear_acceleration().y();
					az = protoPubRole.s_roles()[i].linear_acceleration().z();
					obstacle.acc = std::sqrt(ax * ax + ay * ay + az * az);

					double qx, qy, qz, qw;
					qx = protoPubRole.s_roles()[i].box().rotation().x();
					qy = protoPubRole.s_roles()[i].box().rotation().y();
					qz = protoPubRole.s_roles()[i].box().rotation().z();
					qw = protoPubRole.s_roles()[i].box().rotation().w();
					obstacle.theta = GetYawfromQuaternion(qx, qy, qz, qw);

					if (fabs(obstacle.speed) < 0.01)
					{
						obstacle.is_static = true;
					}
					else
					{
						obstacle.is_static = false;
					}

					switch (protoPubRole.s_roles()[i].type())
					{
					case vts::protocol::main::RoleType::RT_PEDESTRIAN:
						obstacle.roleType = localPlanner::RoleType::PEDESTRIAN;
						break;
					case vts::protocol::main::RoleType::RT_MOTORVEHICLE:
						obstacle.roleType = localPlanner::RoleType::VEHICLE;
						break;
					default:
						obstacle.roleType = localPlanner::RoleType::UNKNOWN;
						break;
					}

					obstacles.push_back(obstacle);
				}
				if (!is_obstacle_init)
				{
					std::cout << "The obstacle is init. " << std::endl; 
					is_obstacle_init = true; 
				}

			}
			
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50hz 
	}
	std::cout << "exit thead." << std::endl;
	return;
}


int main(int argc, char **argv)
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	LogInst->init(); // init LOG 
	LOG_INFO("Hello, I am trajectory generator.");

	// Read map data
	std::shared_ptr<zjlmap::Map> map_data = std::make_shared<zjlmap::Map>();
	ErrorCode ec;
	int handle = 0;
	ec = map_data->load(std::string(FLAGS_map_dir+FLAGS_map_name).c_str(), handle);
	if (ec != ErrorCode::kOK)
	{
		std::cout << "[ERROR] The map load process is failed ! " << std::endl;
		return false;
	}
	
	
	// *************************以下为测试仿真环境SDK接口函数*************************//
	LOG_INFO("[INIT vts] begin...."); 
	vts::drive::VTS_Init(FLAGS_cfg_dir);
	int ret = vts::drive::VTS_Prepare(100); //准备函数，例如，若100秒内仍未准备好则退出程序，入口参数可自设，单位为秒.
	if (ret < 0)
	{
		LOG_ERRO("[INIT vts] fail to init virtual environment!");
		return 0;
		main_exit();
	}
	else
    {
        LOG_INFO("[INIT vts] simulation environment ready!!!" );
    }
	
	std::string case_name = "";
	vts::drive::TARGET_STATE target_info;
	// vts::drive::VTS_GetCaseName(case_name);

	std::cout << "case_name : " << case_name << std::endl;
	LOG_INFO("[Debug] case_name: ", case_name);

	// std::string case_path = FLAGS_case_dir;
    // std::string case_json = case_path + case_name + std::string("/case.json");	
    // std::string anchor_segment_json = case_path + case_name + "/anchor_segment.json";
	vts::drive::VTS_GetCaseInfo(case_name, target_info, role_id); //读取案例信息，role_id为当前案例测试车的ID，用于从PubRole中获取自车位姿、加速度、速度等信息.

	vts::protocol::chassis::VehicleControl protoVehicleControl; // 定义一个控制底盘消息.

	// 初始化底盘控制消息示例.
	protoVehicleControl.mutable_driving_control()->mutable_gear_control()->set_gear_mode(vts::protocol::chassis::GearControlMode::AUTOMATIC_GEAR_MODE);
	protoVehicleControl.mutable_driving_control()->mutable_gear_control()->set_target_gear_position(1);
	protoVehicleControl.mutable_driving_control()->mutable_gear_control()->set_parking(0);
	protoVehicleControl.mutable_driving_control()->mutable_gear_control()->set_clutch_pedal_angle(0);
	protoVehicleControl.mutable_brake_control()->set_target_brake_pedal_position(0);
	protoVehicleControl.mutable_bcm()->set_left_turn_light(0);
	protoVehicleControl.mutable_bcm()->set_right_turn_light(0);
	protoVehicleControl.mutable_bcm()->set_low_beam(1);
	protoVehicleControl.mutable_bcm()->set_high_beam(0);
	protoVehicleControl.mutable_bcm()->set_front_fog_light(0);
	protoVehicleControl.mutable_bcm()->set_rear_fog_light(0);
	protoVehicleControl.mutable_bcm()->set_hazard_warning_light(0);
	protoVehicleControl.mutable_bcm()->set_wiper(2);
	protoVehicleControl.mutable_bcm()->set_horn(0);
	protoVehicleControl.mutable_bcm()->set_auto_beam(0);
	protoVehicleControl.mutable_bcm()->set_auto_wiper(0);
	protoVehicleControl.mutable_bcm()->set_end_outline_marker_lamps(0);

	// Init target generator
	double pdelta = 0.0;
	int counter = 0;
	localPlanner::Pose goal;
	double pre_max_kappa = 0.0;
	std::vector<double> lon_samples = {FLAGS_change_lane_ds};
	std::vector<localPlanner::TargetNode> targets;
	auto ltg = std::make_shared<localPlanner::LocalTargetGenerator>(lon_samples, map_data);
	goal.x = target_info.X;
	goal.y = target_info.Y;
	goal.theta = target_info.Z;
	ltg->UpdateGoal(goal);
	std::cout << "Goal info x: " << goal.x << " y: " << goal.y << " theta: " << goal.theta << std::endl; 

	// Init trajectory planner and controller 
	PlanningOptimizer::TrajectoryPlanner trajectory_planner_handle;
	Controller controller_handle;
	if (!trajectory_planner_handle.Init(map_data))
	{
		std::cout << "[ERROR] The trajectory planner init process is failed! " << std::endl;
		return 0;
	}


	while (1)
	{
		// 接收Notify消息，包括比赛是否开始、终止等信息.
		int ret = vts::drive::VTS_GetNotify(protoNotify);
		if (ret >= 0)
		{
			if (protoNotify.type() == vts::protocol::main::NotifyType::NT_START_TEST)
			{

				if (!start_test)
				{
					start_test = 1;
					thread_ = std::thread(&GetNewPubRole);
					printf("test start \r\n");
				}
			}
			else if (protoNotify.type() == vts::protocol::main::NotifyType::NT_ABORT_TEST || protoNotify.type() == vts::protocol::main::NotifyType::NT_FINISH_TEST)
			{
				if (start_test)
				{
					start_test = 0;
					printf("test complete \r\n");
				}
				break;
			}
			else
			{
				printf("receive notify message, notify type = %d.", protoNotify.type());
			}
		}

		// printf("-------------------ok--------------------------");
		if (start_test)
		{
			unsigned int remaining_number_vehfeedback = 0;
			int ret = vts::drive::VTS_GetVehicleFeedback(protoVehicleFB, vts::drive::GetMessageMode::kLastMessage, remaining_number_vehfeedback); //接收底盘状态信息.
			std::cout << "ret_vehiclefb:" << ret << std::endl;

			if (ret >= 0)
			{
				pdelta = (protoVehicleFB.steering_feedback().steering_wheel_angle() / 100.0)*0.6983;
				fprintf(stderr, "receive vehiclefeedback \r\n");
			}

			std::cout << "[Debug] is robot init: " << is_robot_init << " is_obstacle_init: " << is_obstacle_init << std::endl; 
			if (is_robot_init && is_obstacle_init)
			{				
				// Update robot pose 
				robot.x += (robot.speed)*std::cos(robot.theta)*latency;
				robot.y += (robot.speed)*std::sin(robot.theta)*latency;
				robot.theta += robot.speed / Lf * pdelta*latency;
				double veh_state_linear_speed = robot.speed;
				if (std::abs(veh_state_linear_speed) < 1e-6)
				{
					robot.kappa = 0.0;
				}
				else
				{
					double curr_steering_wheel_angle = protoVehicleFB.steering_feedback().steering_wheel_angle() / 100 * 0.6981;
					robot.kappa = std::tan(curr_steering_wheel_angle) / 1.0;
				}

				std::vector<localPlanner::Box2d> new_obstacles;
				{
					// std::lock_guard<std::mutex> g2(m);
					new_obstacles = obstacles;
				}

				// Targets generating part 
				targets.clear();
				targets = ltg->TargetsGenerating(robot, new_obstacles, pre_max_kappa);

				// Targets debug info 
				std::cout << std::fixed;
				std::cout << std::setprecision(5);
				std::cout << "Behavior targets debug: " << std::endl; 
				for (size_t i = 0; i < targets.size(); i++)
				{
					std::cout << "target " << i << " road_id: " << targets[i].rlsl.lane_id.road_id << " local_id: " << targets[i].rlsl.lane_id.local_id << " s: " << targets[i].rlsl.s <<
						" road_l: " << targets[i].rlsl.l << " lane_l: " << targets[i].sl.l << " laneType: " << targets[i].lane_type << " probo: " << targets[i].driving_prob << std::endl;
					std::cout << "target " << i << " x: " << targets[i].pose.x << " y: " << targets[i].pose.y << std::endl; 
				}

				// Trajectory planning part  
				std::vector<PlanningOptimizer::TrajectoryPoint> planning_trajectory_data;
				if (!trajectory_planner_handle.Process(new_obstacles, robot, targets, planning_trajectory_data))
				{
					std::cout << "[ERROR] The trajectory planner process is failed! " << std::endl;
				}


				double steering_wheel_angle = 0.0;
				double cmd = 0.0;
				if (planning_trajectory_data.size() > 0)
				{
					// Lateral & longitudinal control part  
					controller_handle.LateralControl(robot, planning_trajectory_data, steering_wheel_angle);
					controller_handle.LongitudinalControl(robot, planning_trajectory_data, cmd);
					std::cout << "[Debug Control] steering_wheel_angle: " << steering_wheel_angle << std::endl;
				}
				else
				{
					std::cout << "[ERROR] the planning path is empty ! " << std::endl;
				}

				if (trajectory_planner_handle.trajectory_estop == true)
				{
					std::cout << "[Debug] ESTOP mode. " << std::endl; 
					cmd = -40.0;
				}


				protoVehicleControl.mutable_header()->set_send_ts(protoPubRole.mutable_header()->send_ts());
				protoVehicleControl.mutable_steering_control()->set_target_steering_wheel_angle(steering_wheel_angle); //设定方向盘期望角度为0% 此值在区间[-100, 100]内.
				
				// cmd = 0.0;
				if (cmd > 0)
				{
					protoVehicleControl.mutable_driving_control()->set_target_accelerator_pedal_position(cmd); //设定油门开度为40% 此值在区间[0 100]内.
					protoVehicleControl.mutable_brake_control()->set_target_brake_pedal_position(0.0);
				}
				else
				{
					protoVehicleControl.mutable_driving_control()->set_target_accelerator_pedal_position(0.0); //设定油门开度为40% 此值在区间[0 100]内.
					protoVehicleControl.mutable_brake_control()->set_target_brake_pedal_position(-cmd);
				}


				vts::drive::VTS_PostVehicleControl(protoVehicleControl);//发送控制信号给底盘.
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10hz 
		}
	}

	main_exit();

	return 0;
}


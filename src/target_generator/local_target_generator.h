#pragma once

#include "header.h"
#include "log_manager.h"

namespace localPlanner {

#define M_PI       3.14159265358979323846   // pi

class LocalTargetGenerator {
public:

	LocalTargetGenerator(const std::vector<double>& lon_samples, const std::shared_ptr<zjlmap::Map> Gmap);

	virtual ~LocalTargetGenerator() = default;

	std::vector<TargetNode> TargetsGenerating(Box2d& robot, std::vector<Box2d>& obstacles, const double& pre_max_kappa);

	void UpdateGoal(const Pose& goal);

	bool ReRouting(); // -1 : failed to load global map, 0 : success

	bool TraverseRoutes();   // -1 : need to rerouting, 0 : success

	void AbnormStatusDetection(const double& pre_desired_speed);

	void AddOneHistoryInfo(int event_type);

	HLMRLane_graph GetHLMRLaneGraph();


private:
	//***************method for lon targets sampling******************//
	std::vector<TargetNode> GetLRTargets();
	TargetNode ConstructTarget(const Lane_node& lane_node, const double& s, const double& len_s);
	void ConstructLRTarget(const double& s, const double& len_s, std::vector<TargetNode>& LR_targets);
	std::vector<TargetNode> GetHMTargets();
	int SignofLane(zjlmap::LaneId lane_id);
	bool IsobstacleBlockLane(const Lane_node& lane_node, const Box2d& obs, double& dtheta, double& ds); // dtheta = obs.theta - lane_theta

	void UpdateUGVSL();
	void UpdateCurrentLane();

	std::vector<double> cl_lon_samples_;  // change lane lon target samples
	zjlmap::SLZ goal_sl_;
	Box2d robot_;

	std::shared_ptr<zjlmap::Map> Gmap_;
	zjlmap::Route Gpath_; // Global path;
	zjlmap::LaneId Clane_; // current lane id;
	zjlmap::LaneInfo Clane_info_;
	size_t Clane_index_; // Current land index;     


	bool Is_new_goal_;
	bool Is_new_lane_;

	// lane driving probability related...
	LMRLane_prob travel_prob_;  // traffic rule
	LMRLane_prob guide_prob_; // global path information
	LMRLane_prob lmrlane_prob_;
	void UpdateTravelProb();
	void UpdateGuideProb();
	void UpdateLMRProb();
	bool IsGoalInCurrentRoad();

	std::vector<Box2d> filtered_obstacles_;

	void UpdateFilteredObstacles(const std::vector<Box2d>& obstacles);


	LaneType next_lanetype_;
	void UpdateNextLaneType();

	HLMRLane_graph hlmrlane_graph_;

	void ResetHLMRLaneGraph();
	void UpdateHLMRLaneGraph();
	bool ConstructLaneNode(localPlanner::Lane_node& lane_node);
	bool GetRouteByStartSLZ(const zjlmap::SLZ& start_slz, zjlmap::Route& Gpath);
	double ClampSinLaneInterval(double s, zjlmap::LaneInfo lane_info);

	// Get infomation about current lane e.g. curvature and speed limit
	double lane_max_speed_;
	double lane_curvature_abs_;
	void UpdateLaneBaseInfo();

	// Get lookahead distance
	double lookahead_dist_;
	double pre_max_curvature_;
	void UpdateLookaheadDist();

	// history info
	std::vector<HistoryInfo> history_infos_;
	std::vector<Box2d> static_obstacles_;
	std::vector<Box2d> virtual_obstacles_;
	std::vector<size_t> virtual_obs_indx_;
	void UpdateVirtualObstacles(); // update when drive in new lane
	void InitHistoryInfos();
	void SaveHistoryInfos();
	void UpdateVirtualObstaclesPassInfo(); // update at every decision loop

	double pre_true_speed_;
	double pre_body_z;
	bool is_body_z_init;

	// SIMPLE PREDICTION



	// ¹¦ÄÜº¯Êý
	double Project2LaneLAxis(const double& lane_theta, const Box2d& obs);


};


} // namespace localPlanner

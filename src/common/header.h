#pragma once
#include <memory>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <ctime>
#include <sys/timeb.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include "map.h"

namespace localPlanner{

struct Pointxy
{
    double x;
    double y;
};

struct Pointsl
{
    double s;
    double l;
};

struct Pose
{
    double x;
    double y;
    double theta;
    double kappa;
};
    
struct CubicBezierTrajectory{
  Pointxy p0;
  Pointxy p1;
  Pointxy p2;
  Pointxy p3;

  std::vector<Pose> poses;

  double maxk;
  double k0;
}; 

enum RoleType { VEHICLE, PEDESTRIAN, UNKNOWN}; 

struct TracePoint {
  zjlmap::SLZ rlsl;
  double x;
  double y;
  double z;
  double theta;
  double kappa;
  double speed;
  double acc;
};


struct Box2d{
  std::string id; 
  RoleType roleType; //  -1:others; 0： 机动车； 1：行人； 2：非机动车； 3：动物； 4： 树； 5： 信号灯.

  bool is_static;
  bool is_virtual;

  zjlmap::SLZ rlsl;
  double x;
  double y;
  double z;
  double theta;
  double kappa;
  double speed;
  double acc;  

  double width;
  double length;
};

struct Agent
{
  Box2d box2d;
  size_t past_max_size;
  std::vector<TracePoint> past_points;  // past infomation for intension and prediction
  size_t predict_max_size;
  std::vector<TracePoint> predict_points; 
};


enum LaneType { RIGHT, MIDDLE, LEFT};  

struct TargetNode{
  Pose pose;
  Pointsl sl; // s: length of target, l: offset base on lane center line
  zjlmap::SLZ rlsl; 
  double driving_prob; 
  LaneType lane_type;
};

struct EvaTrajectory{
  TargetNode target;
  CubicBezierTrajectory trajectory;
  double cost;
  double smoothcost;
  double loncost;
  double latcost;
  double collisioncost;
  bool isfeasible;
};

struct  CostCFG
{
    // double w_safe;
    // double w_consist;
    double w_smooth;
    double w_lon;
    double w_lat;
    double w_collsion;
};


struct Trajectory{
  TargetNode target;
  CubicBezierTrajectory trajectory;
};

struct MiddleTarget
{
    zjlmap::SLZ mt_sl;
    double s; // trajectory length
};

 struct LMRLane_prob{
     double l;
     double m;
     double r;
 };  // probability for driving

struct Lane_node{
  bool is_active; // is lane useful?
  zjlmap::LaneInfo lane_info; //lane info
  double ds;
  std::vector<zjlmap::TracePoint> lane_points_info; // start_s --- end_s  ds = 0.3
};


struct HLMRLane_graph{    
    // is lane useful?
    bool is_in_junction;
    Pointxy junction_xy;
    Lane_node L;
    Lane_node M;
    Lane_node R;
    Lane_node HL;
    Lane_node HM;
    Lane_node HR;
    std::vector<Lane_node> Ramp;
    // note: HR and HR aren't used yet
}; 

enum EventType { ROLLED, STICKED, SLIPED}; 

struct HistoryInfo{
  zjlmap::SLZ event_slz;
  double prob;
  double speed_pass;
  double speed_limit;
  int event_type; // 0 ROLLED, 1 STICKED, 2 SLIPED, 3 BUMPED
  bool is_meet;
  bool is_active;
  bool is_pass;
};

// struct LocalView {
//   std::shared_ptr<Box2d> ugv; 
//   std::shared_ptr<std::vector<Box2d> > obstacles;
//   std::shared_ptr<std::vector<localPlanner::TargetNode> > targets;
//   std::shared_ptr<std::vector<localPlanner::Trajectory> > candidate_trajectories;
//   std::shared_ptr<std::vector<localPlanner::EvaTrajectory> > evaluate_trajectories;
//   std::shared_ptr<LocalView> pre_localview;
// }
    
}

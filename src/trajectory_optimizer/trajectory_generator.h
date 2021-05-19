#pragma once
/******************************************************************************
 * Copyright 2020 The zhejiang Lab. Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file trajectory_generator.h
 **/
#include "header.h"
#include "cubic_bezier_curve.h"

namespace localPlanner{

class TrajectoryGenerator{
 public:
   /**
   * fit the cubic bezier curve with four control points
   * @param  p0   the first control point
   */   
    TrajectoryGenerator(const std::vector<double>& lat_samples);

    virtual ~TrajectoryGenerator() = default;
   /**
   * get point (x, y) by input the order of cubic bezier curve's derivative and t
   * @param  order  order of cubic bezier curve's derivative order in [0, 3]
   * @param  t      variable of cubicbezier curve 
   * @return       the point (x, y) on cubic bezier curve at t
   */

   std::vector<Trajectory> CandidateTrajectoryGenerator(const Box2d& robot, const std::vector<TargetNode>& target_nodes);

 private:
   
   std::vector<TargetNode> LatTargetsSampling(const std::vector<TargetNode>& target_nodes);
   
   Pose frenet_to_cartesian(const Pose& pose, const double l);
  
   std::vector<double> lat_samples_; // the lateral traget samples  eg. lane width = 3.75m  {numbers = 20, interval = 0.375}

   std::shared_ptr<CubicBezierCurve> cb_curve_generator_;
};//
}// localplanner namespace

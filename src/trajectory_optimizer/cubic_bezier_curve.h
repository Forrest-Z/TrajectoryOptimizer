#pragma once
/******************************************************************************
 * Copyright 2020 The zhejiang Lab. Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file cubic_bezier_curve.h
 **/

#include "header.h"

namespace localPlanner{

class CubicBezierCurve{
 public:

    CubicBezierCurve();

    virtual ~CubicBezierCurve() = default;
   /**
   * fit the cubic bezier curve with four control points
   * @param  p0   the first control point
   * @param  p1   the second control point
   * @param  p2   the third control point
   * @param  p3   the fourth control point
   */   
    CubicBezierTrajectory GetCubicBezierTrajectory(const Pointxy& p0, const Pointxy& p1, const Pointxy& p2, const Pointxy& p3, const double dt);

  private: 
   /**
   * get point (x, y) by input the order of cubic bezier curve's derivative and t
   * @param  order  order of cubic bezier curve's derivative order in [0, 3]
   * @param  t      variable of cubicbezier curve 
   * @return       the point (x, y) on cubic bezier curve at t
   */  
    Pointxy Evaluate(const int order, const double t);


    double GetCurvatureatInitPoint();


    double GetMaxCurvature();

    double GetMinCurvature();

    std::vector<Pose> GetTrajectoryPoses();

    std::vector<double> GetAllCurvatures();


   /**
   * generate a sequence trajectory points for cubic bezier curve
   * @param  dt  the interval of interpolation
   * @return       the sequence poses (x, y, theta, kappa) on the cubic bezier curve
   */  
   void CubicBezierCurveInterpolation(const double dt);


    Pointxy p0_;
    Pointxy p1_;
    Pointxy p2_;
    Pointxy p3_;
    std::vector<Pose> poses_;
    std::vector<double> abscurvatures_;
    double k0_;
};//
}// localplanner namespace

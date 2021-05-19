/******************************************************************************
 * Copyright 2020 The zhejiang lab Authors. All Rights Reserved.
 *****************************************************************************/
/**
 * @file cubic_bezier_curve.cc
 **/

#include "cubic_bezier_curve.h"


namespace localPlanner
{
    CubicBezierCurve::CubicBezierCurve(){}

    CubicBezierTrajectory CubicBezierCurve::GetCubicBezierTrajectory(const Pointxy& p0, const Pointxy& p1, const Pointxy& p2, const Pointxy& p3, const double dt)
    {
        CubicBezierTrajectory trajectory;
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;

        trajectory.p0 = p0;
        trajectory.p1 = p1;
        trajectory.p2 = p2;
        trajectory.p3 = p3;

        CubicBezierCurveInterpolation(dt);

        trajectory.poses = poses_;
        trajectory.maxk = GetMaxCurvature();
        trajectory.k0 = GetCurvatureatInitPoint();
        return trajectory;
    }

    Pointxy CubicBezierCurve::Evaluate(const int order, const double t)
    {
        Pointxy point;

        switch (order)
        {
        case 0: {
            point.x = (1-t)*(1-t)*(1-t)*p0_.x+3*(1-t)*(1-t)*t*p1_.x+3*(1-t)*t*t*p2_.x+t*t*t*p3_.x;
            point.y = (1-t)*(1-t)*(1-t)*p0_.y+3*(1-t)*(1-t)*t*p1_.y+3*(1-t)*t*t*p2_.y+t*t*t*p3_.y;
            break;
        }
        case 1: {
            point.x = 3*(1-t)*(1-t)*(p1_.x-p0_.x)+6*(1-t)*t*(p2_.x-p1_.x)+3*t*t*(p3_.x-p2_.x);
            point.y = 3*(1-t)*(1-t)*(p1_.y-p0_.y)+6*(1-t)*t*(p2_.y-p1_.y)+3*t*t*(p3_.y-p2_.y);
            break;
        }
        case 2: {
            point.x = 6*(1-t)*(p2_.x-2*p1_.x+p0_.x)+6*t*(p3_.x-2*p2_.x+p1_.x);
            point.y = 6*(1-t)*(p2_.y-2*p1_.y+p0_.y)+6*t*(p3_.y-2*p2_.y+p1_.y);
            break;
        }
        case 3: {
            point.x = -6*p0_.x+18*p1_.x-18*p2_.x+6*p3_.x;
            point.y = -6*p0_.y+18*p1_.y-18*p2_.y+6*p3_.y;
            break;
        }
        default:
            point.x = 0.0;
            point.y = 0.0;
        }
        return point;
    }

    double CubicBezierCurve::GetCurvatureatInitPoint(){return k0_;}

    double CubicBezierCurve::GetMaxCurvature()
    {
        auto maxPosition = std::max_element(abscurvatures_.begin(), abscurvatures_.end());
        return *maxPosition;
    }

    double CubicBezierCurve::GetMinCurvature()
    {
        auto minPosition = std::min_element(abscurvatures_.begin(), abscurvatures_.end());
        return *minPosition;        
    }


    std::vector<Pose> CubicBezierCurve::GetTrajectoryPoses()
    {
        return poses_;
    }

    std::vector<double> CubicBezierCurve::GetAllCurvatures()
    {
        return abscurvatures_;
    }

    void CubicBezierCurve::CubicBezierCurveInterpolation(const double dt)
    {
        poses_.clear();
        abscurvatures_.clear();
        double t = 0.0;
        while (t <= 1.0)
        {
            Pose pose;

            Pointxy p = Evaluate(0, t);
            Pointxy dp = Evaluate(1, t);
            Pointxy ddp = Evaluate(2, t);

            pose.x = p.x;
            pose.y = p.y;
            pose.theta = std::atan2(dp.y, dp.x);
            pose.kappa = (dp.x * ddp.y - dp.y * ddp.x) / pow(dp.x * dp.x + dp.y * dp.y, 1.5);

            poses_.push_back(std::move(pose));
            abscurvatures_.push_back(fabs((dp.x*ddp.y-dp.y*ddp.x)/pow(dp.x*dp.x+dp.y*dp.y ,1.5)));
            if (t == 0.0) 
            {
                k0_ = (dp.x*ddp.y-dp.y*ddp.x)/pow(dp.x*dp.x+dp.y*dp.y ,1.5);
            }

            t += dt;
        }
    }
    
} // localPlanner

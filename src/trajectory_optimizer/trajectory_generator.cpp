/******************************************************************************
 * Copyright 2020 The zhejiang lab Authors. All Rights Reserved.
 *****************************************************************************/
/**
 * @file trajectory_generator.cpp
 **/

#include "trajectory_generator.h"


namespace localPlanner
{    
    TrajectoryGenerator::TrajectoryGenerator(const std::vector<double>& lat_samples)
    {
        lat_samples_ = lat_samples;
        cb_curve_generator_ = std::make_shared<CubicBezierCurve>();
    }

    std::vector<Trajectory> TrajectoryGenerator::CandidateTrajectoryGenerator(const Box2d& robot, const std::vector<TargetNode>& target_nodes)
    {
        std::vector<Trajectory> trajectories;
        std::vector<TargetNode> lattargets = LatTargetsSampling(target_nodes);
        // std::cout<<"targets size:"<<targets.size()<<std::endl;
        Pointxy p0, p1, p2, p3;
        // all candidate trajectories have the same first control point
        p0.x = robot.x;
        p0.y = robot.y;

        for (size_t i = 0; i < lattargets.size(); i++)
        {
            double d = lattargets[i].sl.s/3.0;

            // std::cout<<"d:"<<d<<"; targetx: "<<lattargets[i].pose.x<<"; targety : "<<lattargets[i].pose.y<<std::endl;
            // the second control point 
            p1.x = robot.x + d*std::cos(robot.theta);
            p1.y = robot.y + d*std::sin(robot.theta);

            // the lastthe first control of 
            // the second control point
            p2.x = lattargets[i].pose.x + d*std::cos(lattargets[i].pose.theta + M_PI);
            p2.y = lattargets[i].pose.y + d*std::sin(lattargets[i].pose.theta + M_PI);

            p3.x = lattargets[i].pose.x;
            p3.y = lattargets[i].pose.y;

            Trajectory trajectory;
            trajectory.trajectory = cb_curve_generator_->GetCubicBezierTrajectory(p0, p1, p2, p3, 0.35/lattargets[i].sl.s);
            trajectory.target = lattargets[i];
            
            trajectories.push_back(std::move(trajectory));       
        }
        return trajectories;
    }

    std::vector<TargetNode> TrajectoryGenerator::LatTargetsSampling(const std::vector<TargetNode>& target_nodes)
    {
        std::vector<TargetNode> targetsamples;
        for (size_t i = 0; i < target_nodes.size(); i++)
        {
            TargetNode refer_target_node = target_nodes[i];
            for (size_t j = 0; j < lat_samples_.size(); j++)
            {
                TargetNode tn;
                tn.pose = frenet_to_cartesian(refer_target_node.pose, lat_samples_[j]);
                tn.sl.s = refer_target_node.sl.s;
                tn.sl.l = lat_samples_[j];
                tn.driving_prob = refer_target_node.driving_prob;
                tn.lane_type = refer_target_node.lane_type;
                targetsamples.push_back(std::move(tn));
            }
        }
        return targetsamples;
    }


    Pose TrajectoryGenerator::frenet_to_cartesian(const Pose& refer_pose, const double l)
    {
        Pose p;

        double cos_theta_r = std::cos(refer_pose.theta);
        double sin_theta_r = std::sin(refer_pose.theta);

        p.x = refer_pose.x - sin_theta_r*l;
        p.y = refer_pose.y + cos_theta_r*l;
        p.theta = refer_pose.theta;

        return p;
    }
    
} // localPlanner

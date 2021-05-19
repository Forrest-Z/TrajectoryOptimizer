#pragma once

#include <vector>
#include <algorithm>
#include "map.h"

namespace PlanningOptimizer {

enum TargetLineType
{
	self_lane = 0,
	lane_change = 1
};

class TargetLine
{
public:
	TargetLine() {};
	~TargetLine() {};
	void GetTargetTracePoint(const zjlmap::SLZ& slz_point, zjlmap::TracePoint& target_trace_point);
	zjlmap::TracePoint GetReferencePointFromS(double s);

	std::vector<zjlmap::SLZ> target_line_points;
	std::vector<zjlmap::TracePoint> target_line_trace_points;
	std::vector<double> accumulated_s;
	TargetLineType target_line_type;
};

} // namespace PlanningOptimizer


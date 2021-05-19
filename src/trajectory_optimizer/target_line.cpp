#include "target_line.h"

namespace PlanningOptimizer {

void TargetLine::GetTargetTracePoint(const zjlmap::SLZ& slz_point, zjlmap::TracePoint& target_trace_point)
{
	if (target_line_trace_points.size() == 0)
	{
		return;
	}

	auto comp1 = [](zjlmap::TracePoint trace_p, zjlmap::SLZ slz_p)
	{
		return (trace_p.s >= slz_p.s) && (trace_p.lane_id == slz_p.lane_id);
	};

	auto comp2 = [](zjlmap::TracePoint trace_p, zjlmap::SLZ slz_p)
	{
		return (trace_p.s <= slz_p.s) && (trace_p.lane_id == slz_p.lane_id);
	}; 

	std::vector<zjlmap::TracePoint>::iterator it_lower;
	if (slz_point.lane_id.local_id > 0)
	{
		it_lower = std::lower_bound(target_line_trace_points.begin(), target_line_trace_points.end(), slz_point, comp1);
	}
	else
	{
		it_lower = std::lower_bound(target_line_trace_points.begin(), target_line_trace_points.end(), slz_point, comp2);
	}
	

	if (it_lower == target_line_trace_points.begin())
	{
		target_trace_point = target_line_trace_points.front();
	}
	else if (it_lower == target_line_trace_points.end())
	{
		target_trace_point = target_line_trace_points.back();
	}
	else
	{
		target_trace_point = *it_lower;
	}

	return;

}

zjlmap::TracePoint TargetLine::GetReferencePointFromS(double s)
{
	for (size_t i = 0; i < accumulated_s.size(); i++)
	{
		if (s <= accumulated_s[i])
		{
			return target_line_trace_points[i];
		}
	}

	return target_line_trace_points.back();

}

} // namespace PlanningOptimizer
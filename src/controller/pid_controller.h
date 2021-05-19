#pragma once

#include <iostream>
#include <math.h>

struct PidConf
{
	bool integrator_enable;
	double integrator_saturation_level;
	double kp;
	double ki;
	double kd;
};

class PIDController
{
public:
	PIDController() {};
	~PIDController() {};
	void Init(const PidConf& pid_conf);
	void SetPID(const PidConf& pid_conf);
	void Reset();
	double Control(const double error, const double dt);

private:
	double kp_ = 0.0;
	double ki_ = 0.0;
	double kd_ = 0.0;
	double kaw_ = 0.0;
	double previous_error_ = 0.0;
	double previous_output_ = 0.0;
	double integral_ = 0.0;
	double integrator_saturation_high_ = 0.0;
	double integrator_saturation_low_ = 0.0;
	bool first_hit_ = false;
	bool integrator_enabled_ = false;
	bool integrator_hold_ = false;
	int integrator_saturation_status_ = 0;
	// Only used for pid_BC_controller and pid_IC_controller
	double output_saturation_high_ = 0.0;
	double output_saturation_low_ = 0.0;
	int output_saturation_status_ = 0;

	
};

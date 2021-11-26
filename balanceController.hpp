#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class BalanceController  {
public:
	BalanceController(const Config* settings, Config_PidConfig* pid_config) :
		settings_(settings), d_lpf_(&settings->balance_settings.balance_d_param_lpf_rc), angle_pid_(pid_config), rate_pid_(&settings->yaw_pid) {
		reset();
	}

	void reset() {
		angle_pid_.reset();
		rate_pid_.reset();
		d_lpf_.reset();
		max_D_multiplier_so_far_ = 0;
	}

	float getInput(float angle, float balance_angle) {
		float raw_input = (balance_angle - angle) / settings_->balance_settings.balance_angle_scaling;
		float p_input = constrain(raw_input, -1, 1);

		switch (settings_->balance_settings.expo_type) {
		case 0: return applyExpoReal(p_input, settings_->balance_settings.balance_expo);
		case 1: return applyExpoNatural(p_input, settings_->balance_settings.balance_expo);
		case 2: return applyExpoPoly(p_input, settings_->balance_settings.balance_expo);
		default: while(1);
		}
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is -1:1,
	// but not limited here to that range.
	float compute(float angle, float rate) {
		float rateRequest = angle_pid_.compute(angle);
		return rate_pid_.compute(rateRequest * 100 - rate);
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is -1:1,
	// but not limited here to that range.
	int16_t computeStarting(float angle, float rate, float pid_P_multiplier) {

		rate_pid_.resetI();
		angle_pid_.resetI();
		float rateRequest = angle_pid_.compute(angle);
		rateRequest *= pid_P_multiplier;
		return rate_pid_.compute(rateRequest * 100 - rate);
	}

private:
	const Config* settings_;
	float max_D_multiplier_so_far_ = 0;
	BiQuadLpf d_lpf_;
	PidController angle_pid_;
	PidController rate_pid_;
	int axis_;
};

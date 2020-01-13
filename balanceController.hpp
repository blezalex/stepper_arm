#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class Flotator {
public:
  Flotator() { reset(); }

  void reset() {
    smoothed_out_ = 0;
    prev_value_ = 0;
  }

  float compute(float input, float scaler, float reconciler) {
    float diff = input - prev_value_;
    prev_value_ = input;

    smoothed_out_ += diff * scaler;
    smoothed_out_ = smoothed_out_ * (1 - reconciler) + input * reconciler;
    return smoothed_out_;
  }

private:
  float prev_value_;
  float smoothed_out_;
};

class BalanceController  {
public:
	BalanceController(const Config* settings, Config_PidConfig* pid_config, int axis) :
		settings_(settings), d_lpf_(&settings->balance_settings.balance_d_param_lpf_rc), balance_pid_(pid_config), axis_(axis) {
		reset();
	}

	void reset() {
		balance_pid_.reset();
		d_lpf_.reset();
		max_D_multiplier_so_far_ = 0;
		floatator_.reset();
	}

	float getPIInput(float* angles, float balance_angle) {

		float smootheed_angle = angles[axis_];
		if (settings_->floating.p_scaler < 1) {
			smootheed_angle = floatator_.compute(angles[axis_], settings_->floating.p_scaler, settings_->floating.p_reconcile_rate);
		}
		float raw_input = (balance_angle - smootheed_angle) / settings_->balance_settings.balance_angle_scaling;
		return constrain(raw_input, -1, 1);
	}

	float getPInput(float* angles, float balance_angle) {
		float p_input = getPIInput(angles, balance_angle);
		switch (settings_->balance_settings.expo_type) {
		case 0: return applyExpoReal(p_input, settings_->balance_settings.balance_expo);
		case 1: return applyExpoNatural(p_input, settings_->balance_settings.balance_expo);
		case 2: return applyExpoPoly(p_input, settings_->balance_settings.balance_expo);
		default: while(1);
		}
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is -MOTOR_CMD_RANGE:MOTOR_CMD_RANGE,
	// but not limited here to that range.
	float compute(const int16_t* gyro_update, float* angles, float balance_angle) {
		float avg_gyro_upd = constrain(gyro_update[axis_] * kGyroMultiplier, -settings_->balance_settings.balance_d_param_limiter, settings_->balance_settings.balance_d_param_limiter);
		avg_gyro_upd = d_lpf_.compute(avg_gyro_upd);
		return balance_pid_.compute(getPInput(angles, balance_angle), avg_gyro_upd, getPIInput(angles, balance_angle));
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is -MOTOR_CMD_RANGE:MOTOR_CMD_RANGE,
	// but not limited here to that range.
	int16_t computeStarting(const int16_t* gyro_update, float* angles, float pid_P_multiplier) {
		float pid_D_multiplier =  START_D_MAX_MULTIPLIER *
				min((START_ANGLE_DRIVE - fabs(angles[axis_])) / (START_ANGLE_DRIVE - START_ANGLE_DRIVE_FULL), 1);

		// increase angle compensation as board approaches balance point. Never reduce it until reset
		if (pid_D_multiplier > max_D_multiplier_so_far_)
			max_D_multiplier_so_far_ = pid_D_multiplier;

		int16_t pid_out = balance_pid_.compute(getPInput(angles, 0) * pid_P_multiplier, gyro_update[axis_] * max_D_multiplier_so_far_ * kGyroMultiplier, getPIInput(angles, 0));
		return constrain(pid_out, -START_MAX_POWER, START_MAX_POWER);
	}

private:
	const Config* settings_;
	float max_D_multiplier_so_far_ = 0;
	Flotator floatator_;
	BiQuadLpf d_lpf_;
	PidController balance_pid_;
	int axis_;
	static constexpr float kGyroMultiplier = 1/4.0;
};
#pragma once
#include <math.h>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"
#include "stateTracker.hpp"
#include "lpf.hpp"
#include "drv/vesc/vesc.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"

#define BRAKE_VIA_USART

class ConstrainedOut {
public:
	ConstrainedOut(PwmOut* motor_out, Config_BalancingConfig& balance_settings) :
		motor_out_(motor_out),
		motor_out_lpf_(&(balance_settings.output_lpf_rc)),
		max_update_(&(balance_settings.max_update_limiter)) {}

	void set(float value) {
		float out = constrain(value + NEUTRAL_MOTOR_CMD, MIN_MOTOR_CMD, MAX_MOTOR_CMD);
		float new_out = constrain(out, prev_out_ - *max_update_, prev_out_ + *max_update_);
		new_out = motor_out_lpf_.compute(new_out);

		prev_out_ = new_out;
		motor_out_->set(new_out);
	}

	// sets all to neutral
	void reset() {
		motor_out_lpf_.reset(NEUTRAL_MOTOR_CMD);
		prev_out_ = NEUTRAL_MOTOR_CMD;
	}

private:
	PwmOut* motor_out_;

	uint16_t prev_out_;
	BiQuadLpf motor_out_lpf_;
	int32_t* max_update_;
};


class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, PwmOut& motor_out1,  PwmOut& motor_out2,  PwmOut& motor_out3, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led, VescComm* vesc)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		pitch_balancer_(settings_, &(settings_->pitch_pid), 1),
		roll_balancer_(settings_, &(settings_->roll_pid), 0),
		yaw_pid_controler_(&(settings_->yaw_pid)),
		ppm_motor1_(&motor_out1, settings->balance_settings),
		ppm_motor2_(&motor_out2, settings->balance_settings),
		ppm_motor3_(&motor_out3, settings->balance_settings),
		status_led_(status_led),
		beeper_(beeper),
		green_led_(green_led),

		vesc_(vesc) {
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		State current_state = state_.update();

		switch (current_state) {
		case State::Stopped:
			ppm_motor1_.set(NEUTRAL_MOTOR_CMD);
			ppm_motor2_.set(NEUTRAL_MOTOR_CMD);
			ppm_motor3_.set(NEUTRAL_MOTOR_CMD);

			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			brakes_on_ = false;
			pitch_balancer_.reset();
			roll_balancer_.reset();
			yaw_pid_controler_.reset();

			ppm_motor1_.reset();
			ppm_motor2_.reset();
			ppm_motor3_.reset();

			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
		case State::Running:
			float fwd;
			float right;
			float yaw = yaw_pid_controler_.compute(update.gyro[3]);

			if (current_state == State::Starting){
				fwd = pitch_balancer_.computeStarting(update.gyro, (float*)imu_.angles, state_.start_progress());
				right = roll_balancer_.computeStarting(update.gyro, (float*)imu_.angles, state_.start_progress());
			}
			else {
				fwd = pitch_balancer_.compute(update.gyro, (float*)imu_.angles, 0);
				right = roll_balancer_.compute(update.gyro, (float*)imu_.angles, 0);
			}

		  float v1 = yaw + cos(radians(120)) * right + sin(radians(120)) * fwd;
		  float v2 = yaw + cos(radians(120)) * right - sin(radians(120)) * fwd;
		  float v3 = yaw + right;

			ppm_motor1_.set(v1);
			ppm_motor2_.set(v2);
			ppm_motor3_.set(v3);
			break;
		}

//		if (vesc_update_cycle_ctr_++ >= 50) {
//			// request a stats update every 50 cycles => 20hz
//			vesc_update_cycle_ctr_ = 0;
//			vesc_->requestStats();
//		}
	}


private:
	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController pitch_balancer_;
	BalanceController roll_balancer_;
	PidControllerBasic yaw_pid_controler_;

	ConstrainedOut ppm_motor1_;
	ConstrainedOut ppm_motor2_;
	ConstrainedOut ppm_motor3_;

	GenericOut& status_led_;
	GenericOut& beeper_;

	GenericOut& green_led_;

	uint16_t stopped_since_ts_;
	bool brakes_on_ = false;
	bool first_stopped_to_brake_iteration_ = true;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;
};

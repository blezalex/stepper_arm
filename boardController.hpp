#pragma once
#include <math.h>
#include <limits>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"
#include "stateTracker.hpp"
#include "lpf.hpp"
#include "drv/vesc/vesc.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"

#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_tim.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "io/rx.h"

#define BRAKE_VIA_USART

class StepperOut {

public:
	StepperOut(int idx) : idx_(idx) {
		if (idx >= kMOTOR_CNT) while(1);
		next_iter_completes_step[idx] = false;
		set(0);
	}

	void set(float v_steps_sec) {
		if (fabsf(v_steps_sec) < kMinStepsSec) {
			// Requested speed is too slow. Also protects from division by zero.
			iterations_between_steps[idx_] = std::numeric_limits<uint32_t>::max();
			current_interation[idx_] = 0;
			return;
		}

		uint32_t delay_iters = abs(kSAMPLING_FREQ_HZ / v_steps_sec);
		if (delay_iters < 1) {
			// Max speed reached
			delay_iters = 1;
		}

		iterations_between_steps[idx_] = delay_iters;

		const bool requested_fwd = v_steps_sec > 0;
		if (forward[idx_] != requested_fwd) {
			forward[idx_] = requested_fwd;
			GPIO_WriteBit(kDirPorts[idx_], kDirPins[idx_], (BitAction)requested_fwd);
			current_interation[idx_] = 0;
		}
	}


	static constexpr int32_t kSAMPLING_FREQ_HZ = 100000l;
	static constexpr float kMinStepsSec = (float)kSAMPLING_FREQ_HZ / std::numeric_limits<uint32_t>::max();
	static constexpr int32_t kMOTOR_CNT = 3;


	static uint32_t iterations_between_steps[kMOTOR_CNT];
	static bool forward[kMOTOR_CNT];
	static bool next_iter_completes_step[kMOTOR_CNT];
	static uint32_t current_interation[kMOTOR_CNT];

	static constexpr GPIO_TypeDef* kDirPorts[kMOTOR_CNT] = { GPIOA, GPIOB, GPIOB };
	static constexpr uint16_t kDirPins[kMOTOR_CNT] = { GPIO_Pin_11, GPIO_Pin_7, GPIO_Pin_9 };

	static constexpr GPIO_TypeDef* kStepPorts[kMOTOR_CNT] = { GPIOA, GPIOB, GPIOB };
	static constexpr uint16_t kStepPins[kMOTOR_CNT] = { GPIO_Pin_8, GPIO_Pin_6, GPIO_Pin_8 };

	static void UpdateStates() {
		for (size_t motor_idx = 0; motor_idx < kMOTOR_CNT;  motor_idx++) {
			uint32_t& current_iter = current_interation[motor_idx];
			current_iter++;

			if (next_iter_completes_step[motor_idx]) {
				GPIO_SetBits(kStepPorts[motor_idx], kStepPins[motor_idx]);
				next_iter_completes_step[motor_idx] = false;
				continue;
			}

			uint32_t interval = iterations_between_steps[motor_idx];
//			if (interval == std::numeric_limits<uint32_t>::max()) {
//				// stopped motor;
//				continue;
//			}

			if (current_iter >= interval) {
				// reset step pin
				GPIO_ResetBits(kStepPorts[motor_idx], kStepPins[motor_idx]);

			  // reset counter
				current_iter = 0;
				next_iter_completes_step[motor_idx] = true;
			}
		}
	}

	static void InitAll() {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		GPIO_InitTypeDef  GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);


		// TIMER
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef TimerBaseInit;
		TIM_TimeBaseStructInit(&TimerBaseInit);
		TIM_TimeBaseStructInit(&TimerBaseInit);
		TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
		TimerBaseInit.TIM_Period = 10; // 100kHz
		TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4,&TimerBaseInit);
		TIM_Cmd(TIM4, ENABLE);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	}

private:
	int idx_;
};


extern "C" void TIM4_IRQHandler()
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	}
	StepperOut::UpdateStates();
}

uint32_t StepperOut::iterations_between_steps[kMOTOR_CNT];
uint32_t StepperOut::current_interation[kMOTOR_CNT];
bool StepperOut::forward[kMOTOR_CNT];
bool StepperOut::next_iter_completes_step[kMOTOR_CNT];

constexpr GPIO_TypeDef* StepperOut::kDirPorts[kMOTOR_CNT];
constexpr uint16_t StepperOut::kDirPins[kMOTOR_CNT];

constexpr GPIO_TypeDef* StepperOut::kStepPorts[kMOTOR_CNT];
constexpr uint16_t StepperOut::kStepPins[kMOTOR_CNT];


class ConstrainedOut {
public:
	ConstrainedOut(StepperOut* motor_out, Config_BalancingConfig* balance_settings) :
		settings_(balance_settings),
		motor_out_(motor_out),
		motor_out_lpf_(&(balance_settings->output_lpf_rc)) {
			reset();
		}

	void set(float new_out) {
		float prev_val = motor_out_lpf_.getVal();

		new_out = constrain(new_out, prev_val - settings_->max_update_limiter, prev_val + settings_->max_update_limiter);
		new_out = motor_out_lpf_.compute(new_out);

		motor_out_->set(new_out);
	}

	float get() {
		return motor_out_lpf_.getVal();
	}

	void reset() {
		motor_out_lpf_.reset(0);
		motor_out_->set(0);
	}

private:
	Config_BalancingConfig* settings_;
	StepperOut* motor_out_;
	// This lpf is to smooth out motor output so stepper does not get spikes and does not skip steps.
	BiQuadLpf motor_out_lpf_;
};


class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, StepperOut* motor_out1,  StepperOut* motor_out2, StepperOut* motor_out3, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led, VescComm* vesc)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		pitch_balancer_(settings_, &(settings_->pitch_pid)),
		roll_balancer_(settings_, &(settings_->roll_pid)),
		yaw_pid_controler_(&(settings_->yaw_pid)),
		motor1_(motor_out1, &settings->balance_settings),
		motor2_(motor_out2, &settings->balance_settings),
		motor3_(motor_out3, &settings->balance_settings),
		status_led_(status_led),
		beeper_(beeper),
		green_led_(green_led),
		fwd_lpf_(&settings->misc.throttle_rc),
		right_lpf_(&settings->misc.throttle_rc),
		vesc_(vesc) {
	}

	float mapRcInput(uint16_t input) {
		if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD ) {
			return 0;
		}

		return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		State current_state = state_.update();

		switch (current_state) {
		case State::Stopped:
			motor1_.reset();
			motor2_.reset();
			motor3_.reset();

			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			motor1_.reset();
			motor2_.reset();
			motor3_.reset();

			fwd_lpf_.reset();
			right_lpf_.reset();

			pitch_balancer_.reset();
			roll_balancer_.reset();
			yaw_pid_controler_.reset();

			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
		case State::Running:

			float fwdTargetAngle = mapRcInput(rxVals[1]) * 5;
			float rightTargetAngle = mapRcInput(rxVals[0]) * 5;
			
			//float yaw = yaw_pid_controler_.compute(update.gyro[2])  * state_.start_progress();
			float yaw = mapRcInput(rxVals[3]) * 1500;


			if (current_state == State::Starting){
				fwd = pitch_balancer_.computeStarting(imu_.angles[1] - fwdTargetAngle, update.gyro[1], state_.start_progress());
				right = roll_balancer_.computeStarting(imu_.angles[0] - rightTargetAngle, -update.gyro[0], state_.start_progress());
			}
			else {
				fwd = pitch_balancer_.compute(imu_.angles[1] - fwdTargetAngle, update.gyro[1]);
				right = roll_balancer_.compute(imu_.angles[0] - rightTargetAngle, -update.gyro[0]);
			}

			fwd *= settings_->balance_settings.usart_control_scaling;
			right *= settings_->balance_settings.usart_control_scaling;

		  if (current_state != State::Starting) {
		  	fwd += fwd_lpf_.getVal();
		  	right += right_lpf_.getVal();

		  	fwd_lpf_.compute(fwd);
		  	right_lpf_.compute(right);
		  }

			float speed1 = yaw + right;
			float speed2 = yaw + cos(radians(120)) * right - sin(radians(120)) * fwd;
		  float speed3 = yaw + cos(radians(120)) * right + sin(radians(120)) * fwd;

			motor1_.set(speed1);
			motor2_.set(speed2);
			motor3_.set(speed3);

			break;
		}
	}


public:
	float fwd;
	float right;

	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController pitch_balancer_;
	BalanceController roll_balancer_;
	PidController yaw_pid_controler_;

	ConstrainedOut motor1_;
	ConstrainedOut motor2_;
	ConstrainedOut motor3_;

	GenericOut& status_led_;
	GenericOut& beeper_;

	GenericOut& green_led_;

	// These lpfs compensate for body inertia.
	BiQuadLpf fwd_lpf_;
	BiQuadLpf right_lpf_;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;
};

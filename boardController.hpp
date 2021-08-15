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

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#define BRAKE_VIA_USART

class StepperOut {

public:
	StepperOut(int idx) : idx_(idx) {
		if (idx >= kMOTOR_CNT) while(1);
	}


	void set(float v_steps_sec) {
		if (fabs(v_steps_sec) <  ((float)kSAMPLING_FREQ_HZ / std::numeric_limits<int16_t>::max() )) {
			// Stopped
			iterations_between_steps[idx_] =  -1;
			return;
		}

		bool requested_fwd = v_steps_sec > 0;

		int16_t	delay_iters = abs(kSAMPLING_FREQ_HZ / v_steps_sec);

		if (delay_iters < 1) {
			// Max speed reached
			delay_iters = 1;
		}

		iterations_between_steps[idx_] = delay_iters;
		if (iterations_left[idx_] > delay_iters) {
			iterations_left[idx_] = delay_iters;
		}

		if (forward[idx_] != requested_fwd) {
			forward[idx_] = requested_fwd;
			iterations_left[idx_] = delay_iters;

			if (idx_ == 0) 	{
				GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)requested_fwd);
			}
			else {
				GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)requested_fwd);
			}
			// Raise pin
		}

	}

	static constexpr int16_t kSAMPLING_FREQ_HZ = 20000;
	static constexpr int16_t kMOTOR_CNT = 2;

	static int16_t iterations_between_steps[kMOTOR_CNT];
	static bool forward[kMOTOR_CNT];
	static int16_t iterations_left[kMOTOR_CNT];

	static void UpdateStates() {
		for (size_t motor_idx; motor_idx < kMOTOR_CNT;  motor_idx++) {

			if (iterations_between_steps[motor_idx] == -1) {
				// stopped motor;
				continue;
			}

			if (iterations_left[motor_idx] == 0) {
				// reset step pin, reload
				iterations_left[motor_idx] = iterations_between_steps[motor_idx];

				if (motor_idx == 0) 	{
					GPIO_ResetBits(GPIOA, GPIO_Pin_8);
				}
				else {
					GPIO_ResetBits(GPIOB, GPIO_Pin_6);
				}
			}
			else if (--iterations_left[motor_idx] == 0) {
				// raise step pin.
				if (motor_idx  == 0) 	{
					GPIO_SetBits(GPIOA, GPIO_Pin_8);
				}
				else {
					GPIO_SetBits(GPIOB, GPIO_Pin_6);
				}
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
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);


		// TIMER
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef TimerBaseInit;
		TIM_TimeBaseStructInit(&TimerBaseInit);
		TIM_TimeBaseStructInit(&TimerBaseInit);
		TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
		TimerBaseInit.TIM_Period = 50; // 20kHz
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

int16_t StepperOut::iterations_between_steps[kMOTOR_CNT];
int16_t StepperOut::iterations_left[kMOTOR_CNT];
bool StepperOut::forward[kMOTOR_CNT];


class ConstrainedOut {
public:
	ConstrainedOut(StepperOut* motor_out, Config_BalancingConfig& balance_settings) :
		motor_out_(motor_out),
		motor_out_lpf_(&(balance_settings.output_lpf_rc)), settings_(&balance_settings) {}

	void set(float value) {
		float out = constrain(value, -1, 1);
		float new_out = motor_out_lpf_.compute(new_out);

		// !!!!!!!!!!!!!!!!!!! UCOMMENT ME

		motor_out_->set(settings_->balance_angle_scaling);
//		motor_out_->set(new_out);
	}

	void reset() {
		motor_out_lpf_.reset(0);

		// !!!!!!!!!!!!!!!!!!! UCOMMENT ME

		motor_out_->set(settings_->balance_angle_scaling);
//		motor_out_->set(0);
	}

private:
	StepperOut* motor_out_;

	BiQuadLpf motor_out_lpf_;

	Config_BalancingConfig* settings_;
};


class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, StepperOut* motor_out1,  StepperOut* motor_out2, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led, VescComm* vesc)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		pitch_balancer_(settings_, &(settings_->pitch_pid)),
		roll_balancer_(settings_, &(settings_->roll_pid)),
		yaw_pid_controler_(&(settings_->yaw_pid)),
		motor1_(motor_out1, settings->balance_settings),
		motor2_(motor_out2, settings->balance_settings),
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
			motor1_.set(0);
			motor2_.set(0);

			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			motor1_.reset();
			motor2_.reset();

			pitch_balancer_.reset();
			roll_balancer_.reset();
			yaw_pid_controler_.reset();

			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
		case State::Running:
			float fwd;
			float yaw = yaw_pid_controler_.compute(update.gyro[2])  * state_.start_progress();

			if (current_state == State::Starting){
				fwd = pitch_balancer_.computeStarting(imu_.angles[1], state_.start_progress());
			}
			else {
				fwd = pitch_balancer_.compute(imu_.angles[1], 0);
			}

			if (fwd < 0) {
				// The steering is inverted when going backwards
				yaw *= -1;
			}

		  float v1 = fwd + yaw;
		  float v2 = fwd - yaw;

			motor1_.set(v1);
			motor2_.set(v2);
			break;
		}
	}


private:
	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController pitch_balancer_;
	BalanceController roll_balancer_;
	PidController yaw_pid_controler_;

	ConstrainedOut motor1_;
	ConstrainedOut motor2_;

	GenericOut& status_led_;
	GenericOut& beeper_;

	GenericOut& green_led_;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;
};

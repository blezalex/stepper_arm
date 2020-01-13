#ifndef _PID_H
#define _PID_H
#include "global.h"
#include "drv/comms/protocol.pb.h"

class PidController {
public:
	PidController(const Config_PidConfig* params)
		: _params(params) {
	    reset();
	}

	float compute(float error, float de, float ierror) {
		ierror =  constrain(ierror, -_params->max_i, _params->max_i);
		ierror = ierror * _params->i / 100; // scale it up by I, and by 100 to get close to old configs
		_sumI +=  ierror;
		_sumI = constrain(_sumI, -1, 1); // limit to range

		return  error * _params->p + de * _params->d + applyExpoPoly(_sumI,_params->i_expo)  * MOTOR_CMD_RANGE;
	}

	void reset() {
		_sumI = 0;
	}

private:
	const Config_PidConfig* _params;
	float _sumI;

	DISALLOW_COPY_AND_ASSIGN(PidController);
};


class PidControllerBasic {
public:
	PidControllerBasic(const Config_PidConfig* params) {
		_params = params;
		reset();
	}

	float compute(float error)
	{
		return compute(error, error - _prevError);
	}

	float compute(float error, float de)
	{
		float out = error * _params->p + de * _params->d + _sumI * _params->i;

		_prevError = error;
		_sumI = constrain(_sumI + error, -_params->max_i, _params->max_i);

		return out;
	}

	void reset()
	{
		_sumI = 0;
		_prevError = 0;
	}

private:
	const Config_PidConfig* _params;
	float _prevError;
	float _sumI;
};

#endif

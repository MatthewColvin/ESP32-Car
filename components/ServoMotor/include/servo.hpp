#pragma once

#include <stdio.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include "driver/mcpwm_prelude.h"

class ServoMotor
{
public:
	static constexpr auto MIN_DEGREE = -90;
	static constexpr auto MAX_DEGREE = 90;

	ServoMotor(gpio_num_t servoPin);
	~ServoMotor();
	void setAngle(int angle);
	int getAngle() { return mAngle; }

private:
	static constexpr auto MIN_PULSEWIDTH_US = 500;	// Minimum pulse width in microsecond
	static constexpr auto MAX_PULSEWIDTH_US = 2500; // Maximum pulse width in microsecond

	void attach();
	uint32_t angleToPulseWidth(uint32_t anAngle);

	gpio_num_t mPin;
	mcpwm_cmpr_handle_t mComparator;
	int mAngle = 0;
};

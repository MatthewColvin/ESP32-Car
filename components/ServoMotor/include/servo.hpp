#pragma once

#include <stdio.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

class ServoMotor
{
public:
	static constexpr auto MAX_DEGREE = 180;

	ServoMotor(gpio_num_t servoPin);
	~ServoMotor();
	void setAngle(int angle);
	int getAngle() { return mAngle; }

private:
	static constexpr auto MIN_PULSE = 500;
	static constexpr auto MAX_PULSE = 2500;

	void attach();
	uint32_t map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax);

	gpio_num_t pin;
	ledc_channel_t mChannel;
	ledc_timer_t mTimer;
	int mAngle = 0;
};

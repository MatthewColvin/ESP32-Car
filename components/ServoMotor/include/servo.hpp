#pragma once

#include <stdio.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

class ServoMotor {
public:
	ServoMotor(gpio_num_t servoPin, uint32_t minPulse, uint32_t maxPulse, uint32_t maxDeg);

	void attach();
	void write(int angle);

private:
	uint32_t map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax);

	gpio_num_t pin;
	uint32_t minPulseWidth;
	uint32_t maxPulseWidth;
	uint32_t maxDegree;
};

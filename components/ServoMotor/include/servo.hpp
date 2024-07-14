#pragma once

#include "driver/mcpwm_prelude.h"
#include "mocute052.hpp"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <memory>
#include <stdio.h>

class ServoMotor {
public:
  static constexpr auto MIN_DEGREE = -90;
  static constexpr auto MAX_DEGREE = 90;

  ServoMotor(gpio_num_t servoPin);
  virtual ~ServoMotor();
  void setAngle(int angle);
  int getAngle() { return mAngle; }
  void incrementAngle(int numDegrees);
  void decrementAngle(int numDegrees);
  void controlWith(std::shared_ptr<Mocute052> aController);

private:
  static constexpr auto MIN_PULSEWIDTH_US =
      500; // Minimum pulse width in microsecond
  static constexpr auto MAX_PULSEWIDTH_US =
      2500; // Maximum pulse width in microsecond

  void attach();
  uint32_t angleToPulseWidth(uint32_t anAngle);

  gpio_num_t mPin;

  mcpwm_timer_handle_t mTimer;
  mcpwm_oper_handle_t mOperator;
  mcpwm_cmpr_handle_t mComparator;
  mcpwm_gen_handle_t mGenerator;

  int mAngle = 0;
};

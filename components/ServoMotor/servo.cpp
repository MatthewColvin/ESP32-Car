#include "servo.hpp"
#include "driver/mcpwm_gen.h"
#include "esp_log.h"

#define LOG_TAG "Servo"

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO                     // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

ServoMotor::ServoMotor(gpio_num_t servoPin)
    : mPin(servoPin)
{
    attach();
};

uint32_t ServoMotor::angleToPulseWidth(uint32_t value)
{
    return (value - ServoMotor::MIN_DEGREE) * (ServoMotor::MAX_PULSEWIDTH_US - ServoMotor::MIN_PULSEWIDTH_US) / (ServoMotor::MAX_DEGREE - ServoMotor::MIN_DEGREE) + ServoMotor::MIN_PULSEWIDTH_US;
}

void ServoMotor::attach()
{
    // Create timer
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config;
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
    timer_config.flags.update_period_on_empty = true;
    timer_config.flags.update_period_on_sync = true;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Create Operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config;
    operator_config.group_id = 0;
    operator_config.flags.update_gen_action_on_tez = true;
    operator_config.flags.update_gen_action_on_tep = true;
    operator_config.flags.update_gen_action_on_sync = true;
    operator_config.flags.update_dead_time_on_tez = true;
    operator_config.flags.update_dead_time_on_tep = true;
    operator_config.flags.update_dead_time_on_sync = true;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Connect timer and operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // Create comparator and generator from the operator
    mcpwm_comparator_config_t comparator_config;
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &mComparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config;
    generator_config.gen_gpio_num = mPin;
    generator_config.flags.invert_pwm = false;
    generator_config.flags.io_loop_back =false;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Set inital angle to zero
    setAngle(ServoMotor::MIN_DEGREE);

    // Set generator action on timer and compare event
    // go high on counter empty
    mcpwm_gen_timer_event_action_t actionEvent;
    actionEvent.action = MCPWM_GEN_ACTION_HIGH;
    actionEvent.direction = MCPWM_TIMER_DIRECTION_UP;
    actionEvent.event = MCPWM_TIMER_EVENT_EMPTY;
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator, actionEvent));

    // go low on compare threshold
    mcpwm_gen_compare_event_action_t compareAction;
    compareAction.action = MCPWM_GEN_ACTION_LOW;
    compareAction.comparator = mComparator;
    compareAction.direction = MCPWM_TIMER_DIRECTION_UP;
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator, compareAction));

    // Enable and start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void ServoMotor::setAngle(int angle)
{
    if (angle > MAX_DEGREE || angle < MIN_DEGREE)
    {
        ESP_LOGE(LOG_TAG, "ERROR: Angle must be between %d and %d", MIN_DEGREE, MAX_DEGREE);
        return;
    }
    // ESP_LOGI(LOG_TAG, "Setting Angle: %d", angle);
    auto pulseWidth = angleToPulseWidth(angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mComparator, pulseWidth));
    mAngle = angle;
}

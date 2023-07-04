#include "led.hpp"
#include "esp_log.h"

#define PWM_FREQ 5000 // PWM frequency in Hz
#define LOG_TAG "LED"

template <unsigned size>
constexpr std::array<bool, size> init_array_true()
{
    std::array<bool, size> anArray;
    for (int i = 0; i < size; i++)
    {
        anArray[i] = true;
    }
    return anArray;
};

std::array<bool, LEDC_CHANNEL_MAX> LED::channelAvailability = init_array_true<LEDC_CHANNEL_MAX>();
std::array<bool, LEDC_TIMER_MAX> LED::timerAvailability = init_array_true<LEDC_TIMER_MAX>();

LED::LED(gpio_num_t aPin)
    : mPin(aPin), mBrightness(0)
{
    initialize();
}

LED::~LED()
{
    releaseChannel(mChannel);
    releaseTimer(mTimer);
}

void LED::initialize()
{
    mChannel = getAvailableChannel();
    mTimer = getAvailableTimer();
    if (mChannel == LEDC_CHANNEL_MAX || mTimer == LEDC_TIMER_MAX)
    {
        ESP_LOGE(LOG_TAG, "Failed to set up LED on PIN %d due to lack of timer or channel.", mPin);
    }

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  // High-speed mode
        .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit duty resolution for PWM
        .timer_num = mTimer,                 // Timer number
        .freq_hz = PWM_FREQ,                 // PWM frequency
        .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM mChannel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = mPin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = mChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = mTimer,
        .duty = 0,
        .hpoint = 0,
        .flags = {}};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void LED::setBrightness(uint8_t aNewBrightness)
{
    // ESP_LOGI(LOG_TAG, "Setting Brightness %d", aNewBrightness);
    mBrightness = aNewBrightness;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, mChannel, mBrightness);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, mChannel);
}

uint8_t LED::getBrightness()
{
    return mBrightness;
}

ledc_channel_t LED::getAvailableChannel()
{
    for (int i = 0; i < channelAvailability.size(); i++)
    {
        // ESP_LOGI(LOG_TAG, "Channel %d is %s", i, channelAvailability[i] ? "available" : "in use");
        if (channelAvailability[i])
        {
            channelAvailability[i] = false;
            return static_cast<ledc_channel_t>(i);
        }
    }

    ESP_LOGE(LOG_TAG, "ERROR NO LED CHANNELS LEFT!!!");
    return LEDC_CHANNEL_MAX;
}

ledc_timer_t LED::getAvailableTimer()
{
    for (int i = 0; i < timerAvailability.size(); i++)
    {
        // ESP_LOGI(LOG_TAG, "Timer %d is %s", i, timerAvailability[i] ? "available" : "in use");
        if (timerAvailability[i])
        {
            timerAvailability[i] = false;
            return static_cast<ledc_timer_t>(i);
        }
    }
    ESP_LOGE(LOG_TAG, "ERROR NO LED TIMERS LEFT!!!");
    return LEDC_TIMER_MAX;
}

void LED::releaseTimer(ledc_timer_t aTimer)
{
    if (!timerAvailability[aTimer])
    {
        ESP_LOGI(LOG_TAG, "TIMER %d already available!", aTimer);
    }
    timerAvailability[aTimer] = true;
}

void LED::releaseChannel(ledc_channel_t aChannel)
{
    if (!channelAvailability[aChannel])
    {
        ESP_LOGI(LOG_TAG, "Channel %d already available!", aChannel);
    }
    channelAvailability[aChannel] = true;
}
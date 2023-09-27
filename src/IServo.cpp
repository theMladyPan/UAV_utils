#include "IServo.h"
#include "esp_log.h"
#include "utils.h"


IServo::IServo(uint8_t pin) {
    ServoImpl.attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    _min_us = MIN_PULSE_WIDTH;
    _max_us = MAX_PULSE_WIDTH;
}

IServo::IServo(uint8_t pin, uint32_t min_us, uint32_t max_us) {
    ServoImpl.attach(pin, min_us, max_us);
    _min_us = min_us;
    _max_us = max_us;
}

void IServo::set_angle(float angle) {
    int angle_us = static_cast<int>(mapf(angle, -90, 90, _min_us, _max_us));
    ESP_LOGD("IServo", "Setting servo to %dus", angle_us);
    ServoImpl.write(angle_us);
}

void IServo::set_angle(float angle, float angle_min, float angle_max) {
    if(angle < angle_min) {
        angle = angle_min;
    } else if(angle > angle_max) {
        angle = angle_max;
    }
    int angle_us = static_cast<int>(mapf(angle, -90, 90, _min_us, _max_us));
    ESP_LOGD("IServo", "Setting servo to %dus", angle_us);
    ServoImpl.write(angle_us);
}

void IServo::set_percent(float percent) {
    int percent_us = static_cast<int>(mapf(percent, 0, 100, _min_us, _max_us));
    ESP_LOGD("IServo", "Setting servo to %dus", percent_us);
    ServoImpl.write(percent_us);
}
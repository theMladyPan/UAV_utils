#include "Control/Remote.h"


Remote::Remote(){
    Serial2.end();
    ESP_LOGI("Remote", "Initializing IBus");
    _ibus.begin(Serial2, 0, 32, 16); // Use hardware serial 2 for IBus, TIMER 3
}


void Remote::update() {
    _desired_orientation[0] = pulse_width_to_angle(_ibus.readChannel(0));  // roll
    _desired_orientation[1] = pulse_width_to_angle(_ibus.readChannel(1));  // pitch
    _desired_orientation[2] = pulse_width_to_angle(_ibus.readChannel(3));  // yaw
    _desired_throttle = pulse_width_to_throttle(_ibus.readChannel(2));  // throttle
    _svc = pulse_width_to_throttle(_ibus.readChannel(4));
    _vrb = pulse_width_to_throttle(_ibus.readChannel(5));
}


float Remote::pulse_width_to_angle(uint16_t pulse_width) {
    // measured and found that the angle is -90 at 1000us and 90 at 2000us
    return failsafe(map(pulse_width, 1000, 2000, -90, 90), 0, -90, 90);
}


float Remote::pulse_width_to_throttle(uint16_t pulse_width) {
    // measured and found that the throttle is 0 at 1000us and 100 at 2000us
    return failsafe(map(pulse_width, 1000, 2000, 0, 100), 0, 0, 100);
}


float Remote::get_switch() {
    return _svc;
}


float Remote::get_potentiometer() {
    return _vrb;
}


float Remote::failsafe(float value, float failsafe_val, float min, float max) {
    if (value < min || value > max) {
        return failsafe_val;
    }
    else {
        return value;
    }
}
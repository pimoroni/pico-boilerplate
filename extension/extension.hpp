#pragma once
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"

using namespace motor;
using namespace encoder;

class extension
{
private:
    const int EXTENSION_LIMIT_PIN = 19;
    const pin_pair motor_pins = motor2040::MOTOR_A;
    const pin_pair encoder_pins = motor2040::ENCODER_A;
    const Direction DIRECTION = NORMAL_DIR;
    constexpr float GEAR_RATIO = 98.0f;
    Button extension_limit(EXTENSION_LIMIT_PIN);
    PID pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);

public:
    extension(/* args */);
    ~extension();
};

extension::extension(/* args */)
{
}

extension::~extension()
{
}

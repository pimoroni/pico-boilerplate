#pragma once
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"

class extension
{
public:

  extension(motor::Motor*     mot,
            encoder::Encoder* enc,
            pimoroni::Button* endstop,
            pimoroni::PID*    pos_pid,
            pimoroni::PID*    vel_pid
            );

  enum state
  {
    DISABLED = 0,
    IDLE     = 1,
    HOMING   = 2,
    MOVING   = 3,
    LIMIT    = 4,
    STUCK    = 5
  };

  enum Control
  {
    SPEED    = 0,
    POSITION = 1,
    VELOCITY = 2
  };

  enum Commands
  {
    ENABLE       = 0,
    CONTROL      = 1,
    SET_SPEED    = 2,
    SET_POSITION = 3,
    SET_VELOCITY = 4,
    HOME         = 5
  };

  void update();

  void execute_command(int com, float value);

  int32_t cap_count();
  
  ~extension();

private:

  motor::Motor*     _mot;
  encoder::Encoder* _enc;
  pimoroni::Button* _endstop;
  pimoroni::PID*    _pos_pid;
  pimoroni::PID*    _vel_pid;

  int _current_state = DISABLED;
  int _control_approach = SPEED;
  bool _homing_flag = 0;

  encoder::Encoder::Capture _cap;

  float _speed = 0.0f;

  void _homing();
};


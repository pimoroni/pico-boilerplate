#pragma once
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"

class twist
{
public:

  twist(motor::Motor*     mot_right,
        motor::Motor*     mot_left,
        encoder::Encoder* enc_right,
        encoder::Encoder* enc_left,
        pimoroni::Button* runout_right,
        pimoroni::Button* runout_left,
        pimoroni::Button* endstop,
        pimoroni::PID*    pos_pid_right,
        pimoroni::PID*    vel_pid_right,
        pimoroni::PID*    pos_pid_left,
        pimoroni::PID*    vel_pid_left
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
  
  ~twist();

private:

  motor::Motor*     _mot_right;
  motor::Motor*     _mot_left;
  encoder::Encoder* _enc_right;
  encoder::Encoder* _enc_left;
  pimoroni::Button* _runout_right;
  pimoroni::Button* _runout_left;
  pimoroni::Button* _endstop;
  pimoroni::PID*    _pos_pid_right;
  pimoroni::PID*    _vel_pid_right;
  pimoroni::PID*    _pos_pid_left;
  pimoroni::PID*    _vel_pid_left;

  int _current_state = DISABLED;
  int _control_approach = SPEED;
  bool _homing_flag = 0;

  encoder::Encoder::Capture _cap_right;
  encoder::Encoder::Capture _cap_left;

  float _speed_right = 0.0f;
  float _speed_left = 0.0f;

  void _homing();
};


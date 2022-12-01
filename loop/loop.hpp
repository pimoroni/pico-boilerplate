#pragma once
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "analog.hpp"
#include "pid.hpp"


class loop
{
public:

  loop(motor::Motor*     mot,
       encoder::Encoder* enc,
       pimoroni::Button* runout,
       pimoroni::Analog* force,
       pimoroni::PID*    pos_pid,
       pimoroni::PID*    vel_pid,
       pimoroni::PID*    frc_pid
       );

  enum state
  {
    DISABLED = 0,
    IDLE     = 1,
    HOMING   = 2,
    MOVING   = 3,
    LIMIT    = 4,
    STUCK    = 5,
    GRIPPING = 6
  };

  enum Control
  {
    SPEED    = 0,
    POSITION = 1,
    VELOCITY = 2,
    FORCE   = 3
  };

  enum Commands
  {
    ENABLE       = 0,
    CONTROL      = 1,
    SET_SPEED    = 2,
    SET_POSITION = 3,
    SET_VELOCITY = 4,
    HOME         = 5,
    SET_FORCE    = 6,
  };

  void update();

  void execute_command(int com, float value);

  int32_t get_position();
  int32_t get_velocity();
  float get_force();
  int get_status();
  int get_control();
  
  ~loop();

private:

  motor::Motor*     _mot;
  encoder::Encoder* _enc;
  pimoroni::Button* _runout;
  pimoroni::Analog* _force;
  pimoroni::PID*    _pos_pid;
  pimoroni::PID*    _vel_pid;
  pimoroni::PID*    _frc_pid;

  int _current_status = DISABLED;
  int _control_approach = SPEED;
  int _homing_flag = 0;

  encoder::Encoder::Capture _cap;

  float _speed = 0.0f;
  float _accel = 0.0f;

  float _force_threshold = 0;

  int32_t _cable_end = 0;
  int32_t _cable_length = 0;

  void _homing();
  void _update_status();
};


#include "extension.hpp"

extension::extension(motor::Motor*     mot,
                     encoder::Encoder* enc,
                     pimoroni::Button* endstop,
                     pimoroni::PID*    pos_pid,
                     pimoroni::PID*    vel_pid
                    )
{
    _mot = mot;
    _enc = enc;

    _endstop = endstop;

    _pos_pid = pos_pid;
    _vel_pid = vel_pid;

    _mot->init();
    _enc->init();
    
}

extension::~extension()
{
}

void extension::update()
{
    _cap = _enc->capture();
    bool es_raw = _endstop->raw();
    bool es_read = _endstop->read();

    _homing();

    switch (_control_approach)
    {
    case SPEED:
        break;
    case POSITION:
        _speed = _pos_pid->calculate(_cap.count());
        _mot->speed(_speed);
        break;
    case VELOCITY:
        float accel = _vel_pid->calculate(_cap.revolutions_per_second());
        _speed =+ accel;
        _mot->speed(_speed);
        break;
    }
}

void extension::execute_command(int com, float value)
{
  int val = (int)value;
  switch (com)
  {
  case ENABLE:
    if (val)
    {
      _mot->enable();
    }else{
      _mot->disable();
      _current_state = DISABLED;
    }
    break;
  case CONTROL:
    
    if (SPEED <= val && val <= VELOCITY)
      {
        _control_approach = val;
      }
    break;
  case SET_SPEED:
    _mot->speed(value);
    break;
  case SET_POSITION:
    _pos_pid->setpoint = value;
    break;
  case SET_VELOCITY:
    _vel_pid->setpoint = value;
    break;
  case HOME:
    if (value > 0) 
    {
      value *= -1;
    }
    _vel_pid->setpoint = value;
    _control_approach = VELOCITY;
    _homing_flag = true;
    break;
  default:
    break;
  }
}

void extension::_homing()
{
  if (_homing_flag && !_endstop->raw())
  {
    _control_approach = SPEED;
    _mot->speed(0);
    _enc->zero();
    _homing_flag = 0;
  }
}

int32_t extension::cap_count()
{
    return extension::_cap.count();
}
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

    _homing();

    _update_status();

    if (_current_status != DISABLED)
    {
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
      _mot->stop();
      _mot->disable();
      _homing_flag = 0;
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
    _speed = value;
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
  switch (_homing_flag)
  {
  case 1:
    if (!_endstop->raw())
    {
      _control_approach = POSITION;
      _enc->zero();
      _pos_pid->setpoint = 500;
      _homing_flag++;
    }
    break;
  case 2:
    if (_cap.delta() == 0)
    {
      _enc->zero();
      _homing_flag = 0;
    }
  default:
    break;
  }
}

void extension::_update_status()
{
  bool _mot_is_moving = -0.5 <= _speed && _speed >= 0.5;
  bool _enc_is_moving = -1 <= _cap.delta() && _cap.delta() >= 1;

  if (!_mot->is_enabled())
  {
    _current_status = DISABLED;
  }
  else if (_homing_flag) 
  {
    _current_status = HOMING;
  } 
  else if (!_endstop->raw())
  {
    _control_approach = POSITION;
    _pos_pid->setpoint = _cap.count() + 100;
    _current_status = LIMIT;
  }
  else if (_mot_is_moving && _enc_is_moving)
  {
    _current_status = MOVING;
  }
  else if (!_mot_is_moving && !_enc_is_moving)
  {
    _current_status = IDLE;
  }
  else if (_mot_is_moving && !_enc_is_moving)
  {
    _current_status = STUCK;
  }
}

int32_t extension::get_position()
{
    return extension::_cap.count();
}

int32_t extension::get_velocity()
{
  return _cap.delta();
}

int extension::get_status()
{
  return _current_status;
}

int extension::get_control()
{
  return _control_approach;
}
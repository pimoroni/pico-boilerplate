#include "loop.hpp"

loop::loop(motor::Motor*     mot,
           encoder::Encoder* enc,
           pimoroni::Button* runout,
           pimoroni::Analog* force,
           pimoroni::PID*    pos_pid,
           pimoroni::PID*    vel_pid,
           pimoroni::PID*    frc_pid

           )
{
    _mot = mot;
    _enc = enc;

    _runout = runout;
    _force = force;

    _pos_pid = pos_pid;
    _vel_pid = vel_pid;
    _frc_pid = frc_pid;

    _mot->init();
    _enc->init();   
}

loop::~loop()
{
}

void loop::update()
{
    _cap = _enc->capture();

    _update_status();

    _homing();

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
          _accel = _vel_pid->calculate(_cap.revolutions_per_second());
          _speed =+ _accel;
          _mot->speed(_speed);
          break;
      case FORCE:
          _speed = _frc_pid->calculate(_force->read_voltage());
          _mot->speed(_speed);
          break;
      }
    }
}

void loop::execute_command(int com, float value)
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
  case SET_FORCE:
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

void loop::_homing()
{
  switch (_homing_flag)
  {
    case 1:
      if (!_runout->raw())
      {
        _vel_pid->setpoint = -_vel_pid->setpoint;
        _cable_end = _cap.count();
        _force_threshold = _force->read_voltage() + 0.05;
        _homing_flag++;
      }
      break;
    case 2:
      if (_force->read_voltage() >= _force_threshold)
      {
        _control_approach = POSITION;
        _pos_pid->setpoint = _cap.count() + 500;
        _homing_flag++;
      }
      break;
    case 3:
      if (_cap.delta() == 0)
      {
        _cable_length = _cable_end - _cap.count();
        _enc->zero();
        _pos_pid->setpoint = 0;
        _homing_flag = 0;
      }
      break;
    default:
      break;
  } 
}

void loop::_update_status()
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
  else if (!_runout->raw())
  {
    _control_approach = POSITION;
    _pos_pid->setpoint = _cap.count() - 100;
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
    if (_control_approach == FORCE)
    {
      _current_status = GRIPPING;
    }
    else
    {
    _current_status = STUCK;
    }
  }
}

int32_t loop::get_position()
{
    return loop::_cap.count();
}

int32_t loop::get_velocity()
{
  return _cap.delta();
}

float loop::get_force()
{
  return _force->read_voltage();
}

int loop::get_status()
{
  return _current_status;
}

int loop::get_control()
{
  return _control_approach;
}
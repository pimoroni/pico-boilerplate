#include "twist.hpp"

twist::twist(motor::Motor*     mot_right,
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
             )
{
    _mot_right = mot_right;
    _mot_left = mot_left;
    _enc_right = enc_right;
    _enc_left = enc_left;

    _runout_right = runout_right;
    _runout_left = runout_left;

    _endstop = endstop;

    _pos_pid_right = pos_pid_right;
    _vel_pid_right = vel_pid_right;
    _pos_pid_right = pos_pid_right;
    _pos_pid_left = pos_pid_left;

    _mot_right->init();
    _mot_left->init();
    _enc_right->init();
    _enc_left->init();
    
}

twist::~twist()
{
}

void twist::update()
{
  _cap_right = _enc_right->capture();
  _cap_left = _enc_left->capture();

  _homing();

  if (_current_status != DISABLED)
  {
    switch (_control_approach)
    {
    case SPEED:
        break;
    case POSITION:
        _speed_right = _pos_pid_right->calculate(_cap_right.count());
        _speed_left = _pos_pid_left->calculate(_cap_left.count());
        _mot_right->speed(_speed_right);
        _mot_left->speed(_speed_left);
        break;
    case VELOCITY:
        float accel_right = _vel_pid_right->calculate(_cap_right.revolutions_per_second());
        float accel_left = _vel_pid_left->calculate(_cap_left.revolutions_per_second());
        _speed_right =+ accel_right;
        _speed_left =+ accel_left;
        _mot_right->speed(_speed_right);
        _mot_left->speed(_speed_left);
        break;
    }
  }
}

void twist::execute_command(int com, float value)
{
  int val = (int)value;
  switch (com)
  {
  case ENABLE:
    if (val)
    {
      _mot_right->enable();
      _mot_left->enable();
    }else{
      _mot_right->stop();
      _mot_left->stop();
      _mot_right->disable();
      _mot_left->disable();
      _current_status = DISABLED;
    }
    break;
  case CONTROL:
    if (SPEED <= val && val <= VELOCITY)
      {
        _control_approach = val;
      }
    break;
  case SET_SPEED:
    _mot_right->speed(value);
    _mot_left->speed(-value);
    _speed_right = value;
    _speed_left = -value;
    break;
  case SET_POSITION:
    _target_size = value;
    _pos_pid_right->setpoint = (_target_size + _target_offset) / 2;
    _pos_pid_left->setpoint = (_target_size - _target_offset) / 2;
    break;
  case SET_VELOCITY:
    //_vel_pid->setpoint = value;
    break;
  case HOME:
    if (value > 0) 
    {
      value *= -1;
    }
    _vel_pid_right->setpoint = value;
    _vel_pid_left->setpoint = value;
    _control_approach = VELOCITY;
    _homing_flag = true;
    break;
  case SET_OFFSET:
    _target_offset = value;
    _pos_pid_right->setpoint = (_target_size + _target_offset) / 2;
    _pos_pid_left->setpoint = (_target_size - _target_offset) / 2;
    break;
  default:
    break;
  }
}

void twist::_homing()
{
  switch (_homing_flag)
  {
    case 1:
      if (!_runout_right->raw())
      {
        _vel_pid_right->setpoint = 0;
        _cable_end_right = _cap_right.count();
        _homing_flag++;
      }
      else if (!_runout_left->raw())
      {
        _vel_pid_left->setpoint = 0;
        _cable_end_left = _cap_left.count();
        _homing_flag += 2;
      }
      break;
    case 2:
      if (!_runout_left->raw())
      {
        _vel_pid_right->setpoint = -_vel_pid_left->setpoint;
        _vel_pid_left->setpoint = -_vel_pid_left->setpoint;
        _cable_end_left = _cap_left.count();
        _homing_flag += 2;
      }
      break;
    case 3:
      if (!_runout_right->raw())
      {
        _vel_pid_right->setpoint = -_vel_pid_right->setpoint;
        _vel_pid_left->setpoint = -_vel_pid_right->setpoint;
        _cable_end_right = _cap_right.count();
        _homing_flag++;
      }
      break;
    case 4:
      if (!_endstop->raw())
      {
        _control_approach = POSITION;
        _target_offset = 0;
        _target_size = _cap_right.count() + _cap_left.count() + 500;
        _homing_flag++;
      }
      break;
    case 5:
      if (_cap_right.delta() == 0 && _cap_left.delta() == 0)
      {
        _cable_length = _cable_end_right - _cap_right.count() + _cable_end_left - _cap_left.count();
        _enc_right->zero();
        _enc_left->zero();
        _target_size = 0;
        _homing_flag = 0;
      }
      break;
    default:
      break;
  } 
}

void twist::_update_status()
{
  bool _mot_right_is_moving = -0.5 <= _speed_right && _speed_right >= 0.5;
  bool _enc_right_is_moving = -1 <= _cap_right.delta() && _cap_right.delta() >= 1;
  bool _mot_left_is_moving = -0.5 <= _speed_left && _speed_left >= 0.5;
  bool _enc_left_is_moving = -1 <= _cap_left.delta() && _cap_left.delta() >= 1;

  if (!_mot_right->is_enabled())
  {
    _current_status = DISABLED;
  }
  else if (_homing_flag) 
  {
    _current_status = HOMING;
  } 
  else if (!_runout_right->raw())
  {
    _control_approach = POSITION;
    _pos_pid_right->setpoint = _cap_right.count() - 100;
    _current_status = LIMIT;
  }
  else if (!_runout_left->raw())
  {
    _control_approach = POSITION;
    _pos_pid_left->setpoint = _cap_left.count() - 100;
    _current_status = LIMIT;
  }
  else if (_mot_right_is_moving && _enc_right_is_moving && _mot_left_is_moving && _enc_left_is_moving)
  {
    _current_status = MOVING;
  }
  else if (!_mot_right_is_moving && !_enc_right_is_moving && !_mot_left_is_moving && !_enc_left_is_moving)
  {
    _current_status = IDLE;
  }
  else if ((_mot_right_is_moving && !_enc_right_is_moving) || (_mot_left_is_moving && !_enc_left_is_moving))
  { 
    _current_status = STUCK;
  }
}

int32_t twist::get_size()
{
    return _cap_right.count() + _cap_left.count();
}

int32_t twist::get_offset()
{
    return _cap_right.count() - _cap_left.count();
}

int32_t twist::get_velocity()
{
  return _cap_right.delta();
}

int twist::get_status()
{
  return _current_status;
}

int twist::get_control()
{
  return _control_approach;
}


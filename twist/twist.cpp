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
    bool es_raw = _endstop->raw();
    bool es_read = _endstop->read();

    _homing();

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
      _mot_right->disable();
      _mot_left->disable();
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
    _mot_right->speed(value);
    _mot_left->speed(-value);
    break;
  case SET_POSITION:
    //_pos_pid->setpoint = value;
    break;
  case SET_VELOCITY:
    //_vel_pid->setpoint = value;
    break;
  case HOME:
    if (value > 0) 
    {
      value *= -1;
    }
    //_vel_pid->setpoint = value;
    //_control_approach = VELOCITY;
    //_homing_flag = true;
    break;
  default:
    break;
  }
}

void twist::_homing()
{
  
}

int32_t twist::cap_count()
{
    return twist::_cap_right.count();
}
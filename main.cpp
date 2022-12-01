#include "loopo.hpp"

using namespace motor;
using namespace encoder;

bool update_callback(repeating_timer_t *rt);
bool log_callback(repeating_timer_t *rt);
bool command_callback(repeating_timer_t *rt);

// Create a motor and set its direction and speed scale
Motor m = Motor(MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);

// Create an encoder and set its direction and counts per rev, using PIO 0 and State Machine 0
Encoder enc = Encoder(pio0, 0, ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);

// Create the user button
Button user_sw(motor2040::USER_SW);
Button endstop(19);

// Create PID object for position control
PID pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

extension ext = extension(&m, &enc, &endstop, &pos_pid, &pos_pid);

uint8_t serial_buffer[BUFFER_LENGTH];

repeating_timer_t update_timer;
repeating_timer_t log_timer;
repeating_timer_t command_timer;

int main() 
{
  stdio_init_all();

  add_repeating_timer_ms(UPDATE_RATE * 1000.0f, update_callback, NULL, &update_timer);
  add_repeating_timer_ms(LOG_RATE * 1000.0f, log_callback, NULL, &log_timer);
  add_repeating_timer_ms(LOG_RATE * 1000.0f, command_callback, NULL, &command_timer);

  while(!user_sw.raw()) 
  {}  
}

bool update_callback(repeating_timer_t *rt)
{
  ext.update();
  return 1;
}

bool log_callback(repeating_timer_t *rt)
{
  int32_t p = ext.get_position();
  int s = ext.get_status();
  int c = ext.get_control();
  int32_t d = ext.get_delta();
  printf("status: %d \tcontrol: %d\tpos: %d\tdelta:%d\n", s,c,p,d);
  return 1;
}

bool command_callback(repeating_timer_t *rt)
{
  uint16_t end = read_line(serial_buffer);
  if (end > 1)
  {
    actuator_command message = interpret_buffer(serial_buffer, end);
    printf("ID: %d\tCOMMAND: %d\tVALUE: %f\n", message.id, message.command, message.value);
    ext.execute_command(message.command, message.value);
  }
  return 1;
}
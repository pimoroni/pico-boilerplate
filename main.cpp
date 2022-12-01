#include "loopo.hpp"

using namespace motor;
using namespace encoder;

bool update_callback(repeating_timer_t *rt);
bool log_callback(repeating_timer_t *rt);
bool command_callback(repeating_timer_t *rt);

// Create a motor and set its direction and speed scale
Motor ex_mot = Motor(EX_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);
Motor lp_mot = Motor(LP_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);

// Create an encoder and set its direction and counts per rev, using PIO 0 and State Machine 0
Encoder ex_enc = Encoder(pio0, 0, EX_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);
Encoder lp_enc = Encoder(pio0, 1, LP_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);

// Create the user button
Button user_sw(motor2040::USER_SW);
Button ex_endstop(EX_ENDSTOP_PIN);
Button lp_ronout(lp_RUNOUT_PIN);

Analog force = Analog(FORCE_PIN);

// Create PID object for position control
PID ex_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID ex_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

PID lp_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID lp_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);
PID lp_frc_pid = PID(FRC_KP, FRC_KI, FRC_KD, UPDATE_RATE);

extension ext = extension(&ex_mot, &ex_enc, &ex_endstop, &ex_pos_pid, &ex_pos_pid);
loop lp = loop(&lp_mot, &lp_enc, &lp_ronout, &force, &lp_pos_pid, &lp_vel_pid, &lp_frc_pid);

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
  lp.update();
  return 1;
}

bool log_callback(repeating_timer_t *rt)
{
  int32_t pex = ext.get_position();
  int sex = ext.get_status();
  int cex = ext.get_control();
  int32_t plp = lp.get_position();
  int slp = lp.get_status();
  int clp = lp.get_control();
  float f = lp.get_force();
  printf("ex_status: %d \tex_control: %d\tex_pos: %d\tlp_status: %d \tlp_control: %d\tlp_pos: %d\tforce:%f\n", sex,cex,pex,slp,clp,plp,f);
  return 1;
}

bool command_callback(repeating_timer_t *rt)
{
  uint16_t end = read_line(serial_buffer);
  if (end > 1)
  {
    actuator_command message = interpret_buffer(serial_buffer, end);
    printf("ID: %d\tCOMMAND: %d\tVALUE: %f\n", message.id, message.command, message.value);
    switch (message.id)
    {
    case 0:
      ext.execute_command(message.command, message.value);
      break;
    case 1:
      lp.execute_command(message.command, message.value);
    default:
      break;
    }
    
  }
  return 1;
}
#include "loopo.hpp"

using namespace motor;
using namespace encoder;

bool update_callback(repeating_timer_t *rt);
bool log_callback(repeating_timer_t *rt);
bool command_callback(repeating_timer_t *rt);

// Create a motor and set its direction and speed scale
Motor ex_mot = Motor(EX_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);
Motor tr_mot = Motor(TR_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);
Motor tl_mot = Motor(TL_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);
Motor lp_mot = Motor(LP_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);

// Create an encoder and set its direction and counts per rev, using PIO 0 and State Machine 0
Encoder ex_enc = Encoder(pio0, 0, EX_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);
Encoder tr_enc = Encoder(pio0, 1, TR_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);
Encoder tl_enc = Encoder(pio0, 2, TL_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);
Encoder lp_enc = Encoder(pio0, 3, LP_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);

// Create the user button
Button user_sw(motor2040::USER_SW);
Button ex_endstop(EX_ENDSTOP_PIN);
Button tw_endstop(TW_ENDSTOP_PIN);
Button tr_runout(TR_RUNOUT_PIN);
Button tl_runout(TL_RUNOUT_PIN);
Button lp_ronout(LP_RUNOUT_PIN);

Analog force = Analog(FORCE_PIN);

// Create PID object for position control
PID ex_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID ex_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

PID tr_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID tr_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

PID tl_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID tl_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

PID lp_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID lp_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);
PID lp_frc_pid = PID(FRC_KP, FRC_KI, FRC_KD, UPDATE_RATE);

extension ext = extension(&ex_mot, &ex_enc, &ex_endstop, &ex_pos_pid, &ex_pos_pid);
twist tw = twist(&tr_mot, &tl_mot, &tr_enc, &tl_enc, &tr_runout, &tl_runout, &tw_endstop, &tr_pos_pid, &tr_vel_pid, &tl_pos_pid, &tl_vel_pid);
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

  while(!user_sw.raw()) {}  
}

bool update_callback(repeating_timer_t *rt)
{
  ext.update();
  lp.update();
  tw.update();
  return 1;
}

bool log_callback(repeating_timer_t *rt)
{
  int32_t ex_pos = ext.get_position();
  int ex_sts = ext.get_status();
  int ex_cnt = ext.get_control();

  int32_t tw_pos = tw.get_size();
  int32_t tw_off = tw.get_offset();
  int tw_sts = tw.get_status();
  int tw_cnt = tw.get_control();
  
  int32_t lp_pos = lp.get_position();
  int lp_sts = lp.get_status();
  int lp_cnt = lp.get_control();
  float frc = lp.get_force();

  #ifdef DEBUGGING
  printf("Extension\tpos: %d - sts: %d - cnt: %d\n", ex_pos, ex_sts, ex_cnt);
  printf("Twist\t\tpos: %d - off: %d -sts: %d - cnt: %d\n", tw_pos, tw_off, tw_sts, tw_cnt);
  printf("Loop\t\tpos: %d - frc: %f -sts: %d - cnt: %d\n\n", lp_pos, frc, lp_sts, lp_cnt);
  #else
  printf("%d - %d - %d - %d - %d - %d - %d - %d - %f - %d - %d\n", ex_pos, ex_sts, ex_cnt, tw_pos, tw_off, tw_sts, tw_cnt, lp_pos, frc, lp_sts, lp_cnt);
  #endif
  return 1;
}

bool command_callback(repeating_timer_t *rt)
{
  uint16_t end = read_line(serial_buffer);
  if (end > 1)
  {
    actuator_command message = interpret_buffer(serial_buffer, end);

    #if DEBUGGING
    printf("ID: %d\tCOMMAND: %d\tVALUE: %f\n", message.id, message.command, message.value);
    #endif
    
    switch (message.id)
    {
    case 0:
      ext.execute_command(message.command, message.value);
      break;
    case 1:
      tw.execute_command(message.command, message.value);
      break;
    case 2:
      lp.execute_command(message.command, message.value);
    default:
      break;
    }
    
  }
  return 1;
}
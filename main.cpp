#include <stdio.h>
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"
#include "analog.hpp"

/*
Loop-O firmware for the pimoroni motor2040
to provide control of the actuators and sensors over usb serial.
*/

using namespace motor;
using namespace encoder;

// Enum Definition
enum ControlApproach
{
  NO_CONTROL = 0,
  POSITION_CONTROL = 1,
  VELOCITY_CONTROL = 2,
};

enum Actuators
{
  EXTENSION = 0,
  TWIST_RIGHT = 1,
  TWIST_LEFT = 2,
  LOOP = 3
};

enum Commands
{
  TORQUE_ENABLE = 0,
  CONTROL_APPROACH = 1,
  GOAL_SPEED = 2,
  GOAL_POSITION = 3,
  GOAL_VELOCITY = 4,
  ZERO_ENCODER = 5,
};

// Motor and sensor structs
struct motor_state
{
  int     torque = 0;
  int     control = NO_CONTROL;
  int32_t count = 0;
  int32_t delta = 0;
  float   speed_setpoint = 0.0f;
  float   position_setpoint = 0.0f;
  float   velocity_setpoint = 0.0f;
  float   drive_speed = 0.0f;
  int     motor_status = 0;
};

struct sensors_state
{
  bool  extension = 0;
  bool  twist = 0;
  bool  loop = 0;
  bool  cable0 = 0;
  bool  cable1 = 0;
  bool  cable2 = 0;
  float force = 0.0f;
} sensors;

struct actuator_command
{
  uint8_t id = 0;
  uint16_t command = 0;
  float value = 0.0f;
};

// Timer callback definitions
bool update_callback(repeating_timer_t *rt);
bool log_callback(repeating_timer_t *rt);
bool command_callback(repeating_timer_t *rt);

uint16_t read_line(uint8_t *buffer);
actuator_command interpret_buffer(uint8_t *buffer, uint16_t end_index);
void execute_command();

const int EXTENSION_LIMIT_PIN = 19;
const int TWIST_LIMIT_PIN = 26;
const int CABLE_RUNOUT0_PIN = 16;
const int CABLE_RUNOUT1_PIN = 17;
const int CABLE_RUNOUT2_PIN = 28;
const int SINGLETACT_PIN = motor2040::ADC1;

// Create an array of motor pointers
const pin_pair motor_pins[] = {motor2040::MOTOR_A, motor2040::MOTOR_B,
                               motor2040::MOTOR_C, motor2040::MOTOR_D};
const uint NUM_MOTORS = count_of(motor_pins);
Motor *motors[NUM_MOTORS];

// Create an array of encoder pointers
const pin_pair encoder_pins[] = {motor2040::ENCODER_A, motor2040::ENCODER_B,
                                 motor2040::ENCODER_C, motor2040::ENCODER_D};
const uint NUM_ENCODERS = count_of(encoder_pins);
Encoder *encoders[NUM_ENCODERS];

motor_state motor_states[NUM_MOTORS];

const char *ENCODER_NAMES[] = {"Extension", "Twist Right", "Twist Left", "Loop"};

// The gear ratio of the motor
constexpr float GEAR_RATIO = 98.0f;

// The counts per revolution of the motor's output shaft
constexpr float COUNTS_PER_REV = MMME_CPR * GEAR_RATIO;

// The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
const Direction DIRECTION = NORMAL_DIR;

// The scaling to apply to the motor's speed to match its real-world speed
constexpr float SPEED_SCALE = 5.4f;

// How many times to update the motor per second
const uint UPDATES = 100;
const uint LOGS = 2;
const uint COMMANDS = 20;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;
constexpr float LOG_RATE = 1.0f / (float)LOGS;
constexpr float COMMAND_RATE = 1.0f / (float)COMMANDS;

// PID values
constexpr float POS_KP = 0.14f;  // Position proportional (P) gain
constexpr float POS_KI = 0.0f;   // Position integral (I) gain
constexpr float POS_KD = 0.002f; // Position derivative (D) gain

constexpr float VEL_KP = 30.0f; // Velocity proportional (P) gain
constexpr float VEL_KI = 0.0f;  // Velocity integral (I) gain
constexpr float VEL_KD = 0.4f;  // Velocity derivative (D) gain

// Create the user button
Button extension_limit(EXTENSION_LIMIT_PIN);
Button twist_limit(TWIST_LIMIT_PIN);
Button cable_runout0(CABLE_RUNOUT0_PIN);
Button cable_runout1(CABLE_RUNOUT1_PIN);
Button cable_runout2(CABLE_RUNOUT2_PIN);
Button user_sw(motor2040::USER_SW);

Analog singletact = Analog(SINGLETACT_PIN);

// Create PID object for position and velocity control
PID pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);

const int BUFFER_LENGTH = 512;

uint8_t serial_buffer[BUFFER_LENGTH];

const int SETPOINT_THRESHOLD = 10;

int main()
{
  stdio_init_all();

  for (auto i = 0u; i < NUM_MOTORS; i++)
  {
    motors[i] = new Motor(motor_pins[i], NORMAL_DIR, SPEED_SCALE);
    motors[i]->init();

    encoders[i] = new Encoder(pio0, i, encoder_pins[i], PIN_UNUSED, REVERSED_DIR, COUNTS_PER_REV, true);
    encoders[i]->init();
  }

  repeating_timer_t update_timer;
  repeating_timer_t log_timer;
  repeating_timer_t command_timer;

  add_repeating_timer_ms(UPDATE_RATE * 1000.0f, update_callback, NULL, &update_timer);
  add_repeating_timer_ms(LOG_RATE * 1000.0f, log_callback, NULL, &log_timer);
  add_repeating_timer_ms(LOG_RATE * 1000.0f, command_callback, NULL, &command_timer);

  while (!user_sw.raw()){}

  cancel_repeating_timer(&update_timer);
  cancel_repeating_timer(&log_timer);
  cancel_repeating_timer(&command_timer);

  for (auto i = 0u; i < NUM_MOTORS; i++)
  {
    motors[i]->disable();
  }
}

bool update_callback(repeating_timer_t *rt)
{
  sensors.extension = extension_limit.raw();
  sensors.twist = twist_limit.raw();
  sensors.cable0 = cable_runout0.raw();
  sensors.cable1 = cable_runout1.raw();
  sensors.cable2 = cable_runout2.raw();
  sensors.force = singletact.read_voltage();

  for (auto e = 0u; e < NUM_ENCODERS; e++)
  {
    motor_states[e].count = encoders[e]->count();
    motor_states[e].delta = encoders[e]->delta();
  }

  for (auto e = 0u; e < NUM_MOTORS; e++)
  {
    if ((motor_states[e].delta == 0))
    {
      if (((motor_states[e].count <= (motor_states[e].position_setpoint - SETPOINT_THRESHOLD))||(motor_states[e].count >= (motor_states[e].position_setpoint + SETPOINT_THRESHOLD)))&&(motor_states[e].control!=NO_CONTROL))
      {
        motor_states[e].motor_status = 2;
      }
      else
      {
        motor_states[e].motor_status = 0;
      } 
    }
    else
    {
      motor_states[e].motor_status = 1;
    }
    if (motor_states[e].torque)
    {
      if (!motors[e]->is_enabled())
      {
        motors[e]->enable();
      }
      switch (motor_states[e].control)
      {
      case NO_CONTROL:
        motor_states[e].drive_speed = motor_states[e].speed_setpoint;
        break;
      case POSITION_CONTROL:
        pos_pid.setpoint = motor_states[e].position_setpoint;
        motor_states[e].drive_speed = pos_pid.calculate(motor_states[e].count);
        break;
      case VELOCITY_CONTROL:
        vel_pid.setpoint = motor_states[e].velocity_setpoint;
        float acceleration = pos_pid.calculate(motor_states[e].delta);
        motor_states[e].drive_speed += acceleration;
        break;
      }
      motors[e]->speed(motor_states[e].drive_speed);
    }
    else
    {
      if (motors[e]->is_enabled())
      {
        motors[e]->disable();
      }
    }
  }
  return true;
}

bool log_callback(repeating_timer_t *rt)
{
  printf("%d, %d, %d, %d, %d, %f", sensors.extension, sensors.twist, sensors.cable0, sensors.cable1, sensors.cable2, sensors.force);
  for (auto e = 0u; e < NUM_ENCODERS; e++)
  {
    printf(", %d", motor_states[e].count);
  }
  printf("\n");
  return true;
}


bool command_callback(repeating_timer_t *rt)
{
  uint16_t end = read_line(serial_buffer);
  if (end > 1)
  {
    actuator_command message = interpret_buffer(serial_buffer, end);
    switch (message.command)
      {
      case TORQUE_ENABLE:
        motor_states[message.id].torque = (int)message.value;
        break;
      case CONTROL_APPROACH:
        motor_states[message.id].control = (int)message.value;
        break;
      case GOAL_SPEED:
        motor_states[message.id].speed_setpoint = message.value;
        break;
      case GOAL_POSITION:
        motor_states[message.id].position_setpoint = message.value;
        break;
      case GOAL_VELOCITY:
        motor_states[message.id].velocity_setpoint = message.value;
        break;
      case ZERO_ENCODER:
        encoders[message.id]->zero();
        break;
      }
    end = 0;
    printf("ACK\n");
  }
  end = 0;
  return true;
}

uint16_t read_line(uint8_t *buffer)
{
  uint16_t buffer_index = 0;
  while (true)
  {
    int c = getchar_timeout_us(500);
    if (c != PICO_ERROR_TIMEOUT && buffer_index < BUFFER_LENGTH)
    {
      buffer[buffer_index++] = c;
    }
    else
    {
      break;
    }
  }
  return buffer_index;
}

actuator_command interpret_buffer(uint8_t *buffer, uint16_t end_index)
{
  uint16_t value_length = end_index-1;
  actuator_command message;
  char id = buffer[0];
  char command = buffer[1];
  char value[value_length];
  for (uint16_t i = 0; i < value_length; i++)
  {
    value[i] = buffer[i + 2];
  }
  message.id = id - '0';
  message.command = command - '0';
  message.value = atof(value);
  return message;
}
#pragma once

#include <stdio.h>
#include "pico/stdlib.h"

#include "extension.hpp"
#include "loop.hpp"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"
#include "analog.hpp"

struct actuator_command
{
  uint8_t id = 0;
  uint16_t command = 0;
  float value = 0.0f;
};

using namespace motor;
using namespace encoder;

const pin_pair MOTOR_PINS = motor2040::MOTOR_B;

// The pins of the encoder attached to the profiled motor
const pin_pair ENCODER_PINS = motor2040::ENCODER_B;

// The gear ratio of the motor
constexpr float GEAR_RATIO = 98.0f;

// The counts per revolution of the motor's output shaft
constexpr float COUNTS_PER_REV = MMME_CPR * GEAR_RATIO;

// The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
const Direction DIRECTION_MOTOR = NORMAL_DIR;
const Direction DIRECTION_ENCODER = REVERSED_DIR;

// The scaling to apply to the motor's speed to match its real-world speed
constexpr float SPEED_SCALE = 5.4f;

// How many times to update the motor per second
const uint UPDATES = 100;
const uint LOGS = 10;
const uint COMMANDS = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;
constexpr float LOG_RATE = 1.0f / (float)LOGS;
constexpr float COMMAND_RATE = 1.0f / (float)COMMANDS;

// PID values
constexpr float POS_KP = 0.14f;   // Position proportional (P) gain
constexpr float POS_KI = 0.0f;    // Position integral (I) gain
constexpr float POS_KD = 0.002f;  // Position derivative (D) gain

constexpr float VEL_KP = 30.0f; // Velocity proportional (P) gain
constexpr float VEL_KI = 0.0f;  // Velocity integral (I) gain
constexpr float VEL_KD = 0.4f;  // Velocity derivative (D) gain

const int BUFFER_LENGTH = 512;

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
// This header file declares the helper functions

// File: helper_functions_header.h
// Fall 2025

#pragma once
#include <xc.h>

/********************************************************************
  #define statements for constants
 ********************************************************************/
// Motor pins
#define MOTOR_RIGHT_DIR _LATB12
#define MOTOR_LEFT_DIR _LATA1
#define FORWARD 1
#define BACKWARD 0

// Timer pins

// PWM pins, periods, and duty cycles

/********************************************************************
  Function prototypes
 ********************************************************************/
// Functions to define PWM pins
void config_pwm_right_motor(int duty, int period);
void config_pwm_left_motor(int duty, int period);

// Function to configure A/D in scan mode
void config_ad(void);

// Timer configuration functions can go here
void configureTimer(int time);
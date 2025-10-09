// This header file declares the helper functions from helper_functions_source for the project.

// File: helper_functions_header.h
// Fall 2025

/*
Instructions for use:
- Put this in a new C header file
- Put helper_functions_source.c as a source file
- Include the header file in any C source file that uses these functions
    #include "helper_functions.h"
*/

#include <xc.h>

// --- #define statements for constants --- 
// TODO: set-up functions for pwm, clocks, timers, etc
// Motor pins
#define MOTA_DIR _LATA7

// Timer pins

// PWM pins, periods, and duty cycles

// --- Function prototypes ---
// Function to create a delay
void timer(int time);

// Functions to define PWM pins
void config_pwm1(int duty, int period);
void config_pwm2(int duty, int period);
void config_pwm3(int duty, int period);

// Functions to set up timers
void config_timer1(int time);
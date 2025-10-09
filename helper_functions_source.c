// This is a list of helper functions for the project
//
// File: helper_functions_source.c
// Fall 2025

/*
Instructions for use:
- Put this in a new C source file
- Put helper_functions_header.h as a header file
- Include the header file in any C source file that uses these functions
    #include "helper_functions.h"
*/

#include <xc.h>

/*
 * Function: delay
 * --------------------
 * Configure a delay to run for a certain amount of time.
 *
 * time: The first integer.
 * b: The second integer.
 *
 * returns: void
 */
void timer(int time) {
  delay(time);
}

/*
 * Function: configure_PWM
 * --------------------
 * Configure a PWM channel with a certain period and duty cycle.
 *
 * pin: The first integer.
 * frequency: The second integer.
 * dutyCycle: The third integer.
 *
 * returns: void
 */
void config_pwm1(int duty, int period)
{
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = duty;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = period;             // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC1CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
}


/*
 * Function: timer
 * --------------------
 * Configure a timer to run for a certain amount of time.
 *
 * time: The first integer.
 * b: The second integer.
 *
 * returns: void
 */
void configureTimer(int time) {
    // Configure Timer1
    _TON = 1;           // Turn on timer
    _TCS = 0;           // Internal clock source
    _TCKPS = 0b11;      // Divide-by-256 prescaling

    // Reset Timer1 count to zero initially
    TMR1 = 0;
}
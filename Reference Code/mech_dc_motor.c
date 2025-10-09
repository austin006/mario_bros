// This program drives a DC motor using
// the small red dual h-bridge boards.
//
// It turns CCW while a button attached to pin 5 is depressed,
// and CW while the button is released.
//
// Connections:
//      2:  MotorA direction pin 1
//      3:  MotorA direction pin 2
//      14: MotorA PWM signal (speed control))
//      11:  Button
//
// File:    mech_dc_motor.c
// Author:  Mark Colton
// Date:    9/26/23

#include <xc.h>

#define MTRA_PERIOD 3999    // MotorA PWM period
#define MTRA_IN1 _LATA0     // MotorA direction pin 1 (PIC pin 2)
#define MTRA_IN2 _LATA1     // MotorA direction pin 2 (PIC pin 3)
#define MTRA_DUTY OC1R      // MotorA PWM duty cycle
#define CW 0
#define CCW 1
#define BUTTON _RB7         // Button (PIC pin 11)

// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

// Function prototypes
void config_pwm1(int duty, int period);
void motorA(int direction, int speed);

int main()
{
    // Set up pin 14 for MotorA PWM
    config_pwm1(0, MTRA_PERIOD);
    
    // Configure MotorA direction pins as outputs
    _TRISA0 = 0;
    _TRISA1 = 0;
    
    // Configure Button pin as input
    _TRISB7 = 1;
    
    // Wait and let the PWM do its job behind the scenes
    while(1)
    {
        if(BUTTON == 1)
        {
            motorA(CCW, 100);
        }
        else
        {
            motorA(CW, 40);
        }
    }
        

    return 0;
}

// PWM1 (pin 14) configuration function
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

// Motor function -- Send direction (0 or 1) and speed (0-100%)
void motorA(int direction, int speed)
{
    if (direction == CCW)
    {
        MTRA_IN1 = 1;
        MTRA_IN2 = 0;
    }
    else
    {
        MTRA_IN1 = 0;
        MTRA_IN2 = 1;
    }
    
    MTRA_DUTY = speed/100.0*MTRA_PERIOD;
}

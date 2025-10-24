#include <xc.h>
#include "helper_functions.h"

// Pins 8, 9, and 10 can be a little funky because they are shared with other peripherals.
// Sometimes that messes with our ability to use the pins for analog or digital I/O.
#pragma config OSCIOFNC = OFF // Turn off clock output on pin 8
#pragma config SOSCSRC = DIG // Turn off secondary oscillator on pins 9&10

// 8 MHz oscillator
# pragma config FNOSC = FRCDIV 

// Motor Pin Definitions
#define MOTOR_RIGHT_DIR _LATB12
#define MOTOR_LEFT_DIR _LATA1
#define FORWARD 1
#define BACKWARD 0

// QRD Pin Definitions
#define QRD_LEFT _RA3 //Pin 8
#define QRD_RIGHT _RB2 //Pin 6
#define QRD_CENTER _RA2 //Pin 7

// 8 MHz oscillator
#define PERIOD_1RPS 2599
int speed_right = 2599;
int speed_left = 2599;

//Function Prototype
void robot_init(void);

int main(void) {
    robot_init();

    // Go forward
    MOTOR_LEFT_DIR = FORWARD;
    MOTOR_RIGHT_DIR = FORWARD;
   
    while(1)
    {
        speed_right_motor(speed_right);
        speed_left_motor(speed_left);
       
        speed_right = QRD_LEFT;
        speed_left = QRD_RIGHT;
//        speed_right = PERIOD_1RPS + QRD_LEFT;
//        speed_left = PERIOD_1RPS + QRD_RIGHT;
    }
}

void robot_init(void) {
    // 1 MHz (divide-by-8 postscaler)
    _RCDIV = 0b011;         

    // The PIC24 requires all unused I/O pins to be set to digital outputs set to low when not in use.
    ANSA = 0;
    ANSB = 0;
    TRISA = 0;
    TRISB = 0;
    LATA = 0;
    LATB = 0;
   
    // Configure QRD pins to analog
    config_ad();

    // Configure PWM for motors
    config_pwm_left_motor();
    config_pwm_right_motor();
}
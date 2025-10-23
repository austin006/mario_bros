#include <xc.h>
#include "helper_functions.h"

//# pragma config FNOSC = FRCDIV // 8 MHz oscillator
#pragma config FNOSC = LPFRC //500 kHz low-power internal oscillator
//#pragma config FNOSC = LPRC     // 31 kHz LPRC oscillator

// Pins 8, 9, and 10 can be a little funky because they are shared with other peripherals. 
// Sometimes that messes with our ability to use the pins for analog or digital I/O. 
#pragma config OSCIOFNC = OFF // Turn off clock output on pin 8
#pragma config SOSCSRC = DIG // Turn off secondary oscillator on pins 9&10

/***********************************************************/
// Create an enumerated type for the states
enum {
    line_following,
    ball_pick_up,
    ball_drop_off,
    canyon_navigation,
    lander_laser,
    idle // A safe default state
} state;

/***********************************************************/
// Global variables that define PERCEPTION and initialization
bool SensedBallDetected = false;
bool SensedDropOffZone = false;
bool SensedCanyonAhead = false;
bool SensedLanderLaser = false;
bool SensedBallSecured = false;
bool SensedBallDropped = false;
bool SensedCanyonCleared = false;
bool SensedLanderLaserComplete = false;

/***********************************************************/
// Function prototypes for robot processes
void robot_init(void);
void robot_perception(void);
void robot_planning(void);
void robot_action(void);

/********************************************************************
  Main function - this gets executed at power up, or after a reset
 ********************************************************************/
int main(void) {
    robot_init();
    
    while(1)
    {
        robot_perception();
        robot_planning();
        robot_action();
    }
}

/********************************************************************
  SETUP function - this gets executed before the main loop
 ********************************************************************/
void robot_init(void) {
    state = line_following; // Initial state
    _RCDIV = 0b011;         // 1 MHz (divide-by-8 postscaler)

    // The PIC24 requires all unused I/O pins to be set to digital outputs set to low when not in use. 
    ANSA = 0;
    ANSB = 0;
    TRISA = 0;
    TRISB = 0;
    LATA = 0;
    LATB = 0;

    // Configuration for motor pins
    _TRISA6 = 0;    // Pin 14 -> digital output
    _ANSB0 = 0;     // Pin 04 -> analog off
    _TRISB0 = 0;    // Pin 04 -> digital output 
    _ANSA1 = 0;
    _TRISA1 = 0;
    _ANSB12 = 0;
    _TRISB12 = 0;
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void robot_perception(void) {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages
    
}

/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void robot_planning(void) {
  // The planning FSMs that are used by the robot to perform actions
  // based on the sensing from the Perception stage.
   switch(state) {
    case line_following:
        if (SensedBallDetected) {
            pick_up_ball();
            state = ball_pick_up;
        }
        else if (SensedDropOffZone) {
            drop_off_ball();
            state = ball_drop_off;
        }
        else if (SensedCanyonAhead) {
            navigate_canyon();
            state = canyon_navigation;
        }
        else if (SensedLanderLaser) {
            point_laser();
            state = lander_laser;
        }
        else {
            follow_line();
        }
        break;
    case ball_pick_up:
        if (SensedBallSecured) {
            state = line_following; // return to line following after picking up ball
        }
        break;
    case ball_drop_off:
        if (SensedBallDropped) {
            state = line_following; // return to line following after dropping off ball
        }
        break;
    case canyon_navigation:
        if (SensedCanyonCleared) {
            state = line_following; // return to line following after finishing canyon navigation
        }
        break;
    case lander_laser:
        if (SensedLanderLaserComplete) {
            celebrate(); // do a victory dance
        }
        break;
    default:
        state = line_following; // Error handling: reset to initial state
        break;
   }
}

/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void robot_action(void) {
    // Do the actions decided in robot_planning
    // Each action function is defined in the corresponding task source file
}
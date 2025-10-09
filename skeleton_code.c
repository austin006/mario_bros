// #include <xc.h>
// #include "helper_functions.h"

// Create an enumerated type for the states
enum {line_following, ball_pick_up, ball_drop_off, canyon_navigation, lander_laser} state;

// Function prototypes for robot processes
void robot_init(void);
void robot_perception(void);
void robot_planning(void);
void robot_action(void);

int main(void) {
    robot_init();
    
    if (something) {
        
    }
    while(1)
    {
        robot_perception();
        robot_planning();
        robot_action();
    }
}

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void robot_init(void) {
    // Initialize motors, sensors, timers, PWM, etc.
    // This may involve calling functions from helper_functions_header.c
    state = line_following; // Initial state
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
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
   switch(state) {
    case line_following:
        // Logic for line following
        if (/* IR sensor is sensed */) {
            pick_up_ball();
            state = ball_pick_up;
        }
        else if (/* drop-off bins detected */) {
            drop_off_ball();
            state = ball_drop_off;
        }
        else if (/* the line disappears and a wall is detected */) {
            navigate_canyon();
            state = canyon_navigation;
        }
        else if (/* condition to switch to lander_laser */) {
            point_laser();
            state = lander_laser;
        }
        else {
            follow_line();
        }
        break;
    case ball_pick_up:
        // Logic for ball pick up
        if (/*ball is picked up*/) {
            state = line_following; // go back to line following after picking up ball
        }
        break;
    case ball_drop_off:
        // Logic for ball drop off
        if (/*ball is dropped off*/) {
            state = line_following; // go back to line following after dropping off ball
        }
        break;
    case canyon_navigation:
        // Logic for canyon navigation
        if (/* left the canyon or sensed line again */) {
            state = line_following;
        }
        break;
    case lander_laser:
        // Logic for lander laser
        if (/* task is completed */) {
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
}
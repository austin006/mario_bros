/********************************************************************
  ECEN 240/301 Lab Code
  Light-Following Robot

  The approach of this code is to use an architectured that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.

  Version History
  1.1.3       11 January 2023   Creation by Dr. Mazzeo and TAs from 2022 version

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
// your sensors and servos. */
#include "Arduino.h"
#include <CapacitiveSensor.h>
#include <Servo.h>
#include <NewPing.h>

//
// Compiler defines: the compiler replaces each name with its assignment
// (These make your code so much more readable.)
//

/***********************************************************/
// Hardware pin definitions
// Replace the pin numbers with those you connect to your robot

// Button pins. These will be replaced with the photodiode variables in lab 5
// #define BUTTON_1  A2     // Far left Button - Servo Up
// #define BUTTON_2  A3     // Left middle button - Left Motor
#define BUTTON_3  A4     // Middle Button - Collision
// #define BUTTON_4  A5     // Right middle button - Right Motor
// #define BUTTON_5  A6     // Far right button - Servo Down

// LED pins (note that digital pins do not need "D" in front of them)
#define LED_1   6       // Far Left LED - Servo Up
//#define LED_2   5       // Left Middle LED  - Left Motor
#define LED_3   4       // Middle LED - Collision
//#define LED_4   3       // Right Middle LED - Right Motor
#define LED_5   2       // Far Right LED - Servo Down

// Motor enable pins - Lab 3
// These will replace LEDs 2 and 4
#define H_BRIDGE_ENA  5
#define H_BRIDGE_ENB  3

// Photodiode pins - Lab 5
// These will replace buttons 1, 2, 4, 5
#define PHOTO_UP  A2
#define PHOTO_LEFT  A3
#define PHOTO_RIGHT  A5 
#define PHOTO_DOWN  A6 

// Capacitive sensor pins - Lab 4
#define CAP_SENSOR_SEND  11 //cap. sensor send pin
#define CAP_SENSOR_RECEIVE  7 //cap sensor receive pin

// Ultrasonic sensor pin - Lab 6
// This will replace button 3 and LED 3 will no longer be needed
#define TRIGGER_PIN 12
#define ECHO_PIN 10

// Servo pin - Lab 6
// This will replace LEDs 1 and 5
#define SERVO_PIN 9


/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Voltage at which a button is considered to be pressed
#define BUTTON_THRESHOLD 2.5

// Voltage at which a photodiode voltage is considered to be present - Lab 5
#define PHOTODIODE_LIGHT_THRESHOLD 3

// Number of samples that the capacitor sensor will use in a measurement - Lab 4
#define CAP_SENSOR_SAMPLES 40
#define CAP_SENSOR_THRESHOLD 25

// Parameters for servo control as well as instantiation - Lab 6
#define SERVO_START_ANGLE 90
#define SERVO_UP_LIMIT 180
#define SERVO_DOWN_LIMIT 0
static Servo myServo;

// Parameters for ultrasonic sensor and instantiation - Lab 6
// Parameter to define when the ultrasonic sensor detects a collision - Lab 6
#define MAX_DISTANCE 20


/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Motor speed definitions - Lab 4
#define SPEED_STOP    0
#define SPEED_LOW     (int) (255 * 0.33)
#define SPEED_MED     (int) (255 * 0.66)
#define SPEED_HIGH    (int) (255 * 1)

// Collision definitions
#define COLLISION_ON   0
#define COLLISION_OFF  1

// Driving direction definitions
#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

// Servo movement definitions
#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_UP   1
#define SERVO_MOVE_DOWN 2


/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision = DETECTION_NO;

// Photodiode inputs (using Definitions) - The button represent the photodiodes for lab 2
int SensedLightRight = DETECTION_NO;
int SensedLightLeft = DETECTION_NO;
int SensedLightUp = DETECTION_NO;
int SensedLightDown = DETECTION_NO;

// Capacitive sensor input (using Definitions) - Lab 4
int SensedCapacitiveTouch = DETECTION_NO;


/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STOP;
// Add speed action in Lab 4
int ActionRobotSpeed = SPEED_LOW;
// Servo Action (using Definitions)
int ActionServoMove =  SERVO_MOVE_STOP;
// Angle of the servo
int CurrentServoAngle = SERVO_START_ANGLE;

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {
  //Set up serial connection at 9600 Baud
  Serial.begin(9600);
  
  //Set up output pins
  pinMode(LED_1, OUTPUT);
  pinMode(H_BRIDGE_ENA, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(H_BRIDGE_ENB, OUTPUT);
  pinMode(LED_5, OUTPUT);
  
  //Set up input pins
  pinMode(PHOTO_UP, INPUT);
  pinMode(PHOTO_LEFT, INPUT);
  pinMode(BUTTON_3, INPUT);
  pinMode(PHOTO_RIGHT, INPUT);
  pinMode(PHOTO_DOWN, INPUT);

  //Set up capacitive sense pins - Lab 4
  pinMode(CAP_SENSOR_RECEIVE, INPUT);
  pinMode(CAP_SENSOR_SEND, OUTPUT);

  //Set up servo - Lab 6
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);
}

/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/
void loop() {
  // This DebugStateOutput flag can be used to easily turn on the
  // serial debugging to know what the robot is perceiving and what
  // actions the robot wants to take.
  int DebugStateOutput = true; // Change false to true to debug
  
  RobotPerception(); // PERCEPTION
  if (DebugStateOutput) {
    Serial.print("Perception:");
    Serial.print(SensedLightUp);
    Serial.print(SensedLightLeft);
    Serial.print(SensedCollision);
    Serial.print(SensedLightRight); 
    Serial.print(SensedLightDown);
    Serial.print(SensedCapacitiveTouch); //- Lab 4
    Serial.print("\t");
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.print(" Action:");
    Serial.print(ActionCollision);
    Serial.print(ActionRobotDrive); 
    Serial.print(ActionServoMove);
    Serial.print(" "); Serial.print(ActionRobotSpeed); //- Lab 4
    Serial.print("\t");
  }
  RobotAction(); // ACTION
  Serial.print("\n");
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages


  
  // Photodiode Sensing
  //Serial.print(getPinVoltage(BUTTON_2)); Serial.print("\t"); //uncomment for debugging

  if (isLight(PHOTO_LEFT)){
    SensedLightLeft = DETECTION_YES;
  } else {
    SensedLightLeft = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (isLight(PHOTO_RIGHT)){
    SensedLightRight = DETECTION_YES;
  } else {
    SensedLightRight = DETECTION_NO;
  }

      
  /* Add code to detect if light is up or down. Lab 2 milestone 3*/
  if (isLight(PHOTO_UP)){
    SensedLightUp = DETECTION_YES;
  } else {
    SensedLightUp = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (isLight(PHOTO_DOWN)){
    SensedLightDown = DETECTION_YES;
  } else {
    SensedLightDown = DETECTION_NO;
  }
  

   // Capacitive Sensor
   /*Add code in lab 4*/
   if (isCapacitiveSensorTouched()) {
    SensedCapacitiveTouch = DETECTION_YES;
   } else {
    SensedCapacitiveTouch = DETECTION_NO;
   }

   // Collision Sensor
   if (isCollision()) {   // Add code in isCollision() function for lab 2 milestone 1
    SensedCollision = DETECTION_YES;
   } else {
    SensedCollision = DETECTION_NO;
   }
}


////////////////////////////////////////////////////////////////////
// Function to read pin voltage
////////////////////////////////////////////////////////////////////
float getPinVoltage(int pin) {
  //This function can be used for many different tasks in the labs
  //Study this line of code to understand what is going on!!
  //What does analogRead(pin) do?
  //Why is (float) needed?
  //Why divide by 1024?
  //Why multiply by 5?
  return 5 * (float)analogRead(pin) / 1024;
}

////////////////////////////////////////////////////////////////////
// Function to determine if a button is pushed or not
////////////////////////////////////////////////////////////////////
bool isButtonPushed(int button_pin) {
  //This function can be used to determine if a said button is pushed.
  //Remember that when the voltage is 0, it's only close to zero.
  //Hint: Call the getPinVoltage function and if that value is greater
  // than the BUTTON_THRESHOLD variable toward the top of the file, return true.
  if (getPinVoltage(button_pin)>= BUTTON_THRESHOLD){
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if light is present
////////////////////////////////////////////////////////////////////
bool isLight(int pin) {
  float light = getPinVoltage(pin);
  Serial.println(light); // Use this line to test
  return (light > PHOTODIODE_LIGHT_THRESHOLD);
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
static NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

bool isCollision() {
  //This is where you add code that tests if the collision button 
  // was pushed (BUTTON_3)
  //In lab 6 you will add a sonar sensor to detect collision and
  // the code for the sonar sensor will go in this function.
  // Until then we will use a button to model the sensor.
  int sonar_distance = sonar.ping_cm(); // If the distance is too big, it returns 0.
  Serial.println(sonar_distance);
  if(sonar_distance != 0){ 
    return (sonar_distance < MAX_DISTANCE);
  } else {
	return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if the capacitive sensor is being touched
////////////////////////////////////////////////////////////////////
static CapacitiveSensor sensor = CapacitiveSensor(CAP_SENSOR_SEND, CAP_SENSOR_RECEIVE);

bool isCapacitiveSensorTouched() {
  //In lab 4 you will add a capacitive sensor, and
  // you will need to modify this function accordingly.
  long tau = sensor.capacitiveSensor(CAP_SENSOR_SAMPLES);
  if (tau >= 500) {
    return(true);
  }
  else {
    return(false);
  }
}


/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void RobotPlanning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsmCollisionDetection(); // Milestone 1
  fsmMoveServoUpAndDown(); // Milestone 3
  // Add Speed Control State Machine in lab 4
  fsmCapacitiveSensorSpeedControl();
}

////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsmCollisionDetection() {
  static int collisionDetectionState = 0;
  //Serial.print(collisionDetectionState); Serial.print("\t"); //uncomment for debugging
  
  switch (collisionDetectionState) {
    case 0: //collision detected
      //There is an obstacle, stop the robot
      ActionCollision = COLLISION_ON; // Sets the action to turn on the collision LED
      ActionRobotDrive = DRIVE_STOP;
      
      //State transition logic
      if ( SensedCollision == DETECTION_NO) {
        collisionDetectionState = 1; //if no collision, go to no collision state
      }
      break;
    
    case 1: //no collision
      //There is no obstacle, drive the robot
      ActionCollision = COLLISION_OFF; // Sets action to turn off the collision LED

      fsmSteerRobot(); // Milestone 2
      
      //State transition logic
      if (SensedCollision == DETECTION_YES) {
        collisionDetectionState = 0; //if collision, go to collision state
      }
      break;

    default: // error handling
      {
        collisionDetectionState = 0;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is to the right or left,
// and steering the robot accordingly.
////////////////////////////////////////////////////////////////////
void fsmSteerRobot() {
  static int steerRobotState = 0;
  //Serial.print(steerRobotState); Serial.print("\t"); //uncomment for debugging

  switch (steerRobotState) {
    case 0: //light is not detected
      ActionRobotDrive = DRIVE_STOP;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_YES ) {
        steerRobotState = 1; //if light on left of robot, go to left state
      } else if ( SensedLightRight == DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      }
      break;
    
    case 1: //light is to the left of robot
      //The light is on the left, turn left
      ActionRobotDrive =  DRIVE_LEFT;
      
      //State transition logic
      if ( SensedLightRight == DETECTION_YES) {
        steerRobotState = 3; //if light is on right, then go straight
      } else if ( SensedLightLeft == DETECTION_NO ) {
        steerRobotState = 0; //if light is not on left, go back to stop state
      }
      
      break;
    
    case 2: //light is to the right of robot
      //The light is on the right, turn right
      ActionRobotDrive = DRIVE_RIGHT;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_YES) {
        steerRobotState = 3; //if light is on left, then go straight
      } else if ( SensedLightRight == DETECTION_NO ) {
        steerRobotState = 0; //if light is not on right, go back to stop state
      }

      break;
      
    // light is on both right and left
    case 3:
      ActionRobotDrive = DRIVE_STRAIGHT;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_NO) {
        steerRobotState = 2; 
      } else if ( SensedLightRight == DETECTION_NO ) {
        steerRobotState = 1; 
      }

      break;
      
      
    default: // error handling
    {
      steerRobotState = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is above or below center,
// and moving the servo accordingly.
////////////////////////////////////////////////////////////////////
void fsmMoveServoUpAndDown() {
  static int moveServoState = 0;
  //Serial.print(moveServoState); Serial.print("\t"); //uncomment for debugging
    // Milestone 3
  //Create a state machine modeled after the ones in milestones 1 and 2
  // to plan the servo action based off of the perception of the robot
  //Remember no light or light in front = servo doesn't move
  //Light above = servo moves up
  //Light below = servo moves down
  switch(moveServoState){
    case 0://stop case
      ActionServoMove = SERVO_MOVE_STOP;
      //logic
      if(SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_NO) {
        moveServoState = 1;
      } else if(SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_YES) {
        moveServoState = 2;
      }
      break;

    case 1://up case
      ActionServoMove = SERVO_MOVE_UP;
      //logic
      if(SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_YES) {
        moveServoState = 0;
      } else if(SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_NO) {
        moveServoState = 0;
      }
      break;

    case 2://down case
      ActionServoMove = SERVO_MOVE_DOWN;
      //logic
      if(SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_YES) {
        moveServoState = 0;
      } else if(SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_NO) {
        moveServoState = 0;
      }
      break;
  } //end switch
}

////////////////////////////////////////////////////////////////////
// State machine for detecting when the capacitive sensor is
// touched, and changing the robot's speed.
////////////////////////////////////////////////////////////////////
void fsmCapacitiveSensorSpeedControl() {
  /*Implement in lab 4*/
  static int speedControlState = 0;
  switch(speedControlState) {
    case 0: //waiting for touch
      if (SensedCapacitiveTouch == DETECTION_YES) {
        speedControlState = 1; // go to wait for touch end
      } else {
        speedControlState = 0; // continue to wait for touch
      }
    break;
    case 1: //waiting for end of touch
      if (SensedCapacitiveTouch == DETECTION_NO) {
        speedControlState = 2; //go to toggle speed state
      } else {
        speedControlState = 1; // continue to wait for touch to end
      }
    break;
    case 2: //toggle speed
      fsmChangeSpeed();
      speedControlState = 0;
    break;
    default: // error handling 
    {
    speedControlState = 0;
    }
  } // end switch
} //end fsm

////////////////////////////////////////////////////////////////////
// State machine for cycling through the robot's speeds.
////////////////////////////////////////////////////////////////////
void fsmChangeSpeed() {
  /*Implement in lab 4*/
  static int changeSpeedState = 0;

  switch(changeSpeedState) {
    case 0:
    ActionRobotSpeed = SPEED_STOP;
    changeSpeedState = 1;
    break;
    case 1:
    ActionRobotSpeed = SPEED_LOW;
    changeSpeedState = 2;
    break;
    case 2:
    ActionRobotSpeed = SPEED_MED;
    changeSpeedState = 3;
    break;
    default:
    {
      changeSpeedState = 0;
    }
  }
}


/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void RobotAction() {
  // Here the results of planning are implented so the robot does something

  // This turns the collision LED on and off
  switch(ActionCollision) {
    case COLLISION_OFF:
      doTurnLedOff(LED_3); //Collision LED off - DON'T FORGET TO ADD CODE TO doTurnLedOff() 
                           // AND doTurnLedOn() OR ELSE YOUR LEDS WON'T WORK!!!
      break;
    case COLLISION_ON:
      doTurnLedOn(LED_3);
      break;
  }
  
  // This drives the main motors on the robot
  switch(ActionRobotDrive) {
    case DRIVE_STOP:
      /* Add code in milestone 2 to turn off both left and right motors (LEDs right now). 
        Use the doTurnLedOff() function */
      /* DON'T FORGET TO USE YOUR LED VARIABLES AND NOT YOUR BUTTON VARIABLES FOR THIS!!! */
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, 0);
      break;
    case DRIVE_LEFT:
      /* Add code in milestone 2 to turn off the right and on the left LEDs */
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENB, 0);
      break;
    case DRIVE_RIGHT:
      /* Add code in milestone 2 */
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      break;
    case DRIVE_STRAIGHT:
      /* Add code in milestone 2 */
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      break;
  }
  
  // This calls a function to move the servo
    MoveServo();       
}


////////////////////////////////////////////////////////////////////
// Function that causes the servo to move up or down.
////////////////////////////////////////////////////////////////////
void MoveServo() {
  // Note that there needs to be some logic in the action of moving
  // the servo so that it does not exceed its range
  /* Add CurrentServoAngle in lab 6 */
  switch(ActionServoMove) {
    case SERVO_MOVE_STOP: // do nothing
      break;
    case SERVO_MOVE_UP:
      if (CurrentServoAngle <= SERVO_UP_LIMIT) {
        CurrentServoAngle = CurrentServoAngle + 5;
      }
      break;
    case SERVO_MOVE_DOWN:
      if (CurrentServoAngle >= SERVO_DOWN_LIMIT) {
        CurrentServoAngle = CurrentServoAngle - 5;
      }
      break;
  }
  myServo.write(CurrentServoAngle); // send angle to the servo 
}



/**********************************************************************************************************
  AUXILIARY functions that may be useful in performing diagnostics
 ********************************************************************/
// Function to turn LED on
void doTurnLedOn(int led_pin)
{
  digitalWrite(led_pin, HIGH);
}

// Function to turn LED off
void doTurnLedOff(int led_pin)
{
  digitalWrite(led_pin, LOW);
}
# Framework

The code is architectured into three different processes:

- Perception
- Planning
- Action

## Initialization

Some of this will be done in the file `helper_functions_header.c`

```
Setup
    Import necessary libraries
    Hardware pin definitions and functions (motors, sensors, communciation, etc.)
    Configuration parameter definitions (e.g., servo control limits, speed levels, sensor limits)
    Definitions to allow for setting states
    Global variables for each processes

Forever loop
    robotPerception();
    robotPlanning();
    robotAction();
```

## Perception 

All the sensing.

```
robotPerception
    Read all sensors (Distance, color, IR, etc.)
    Store results in "sensedVariables"
    Example:
        sensedObstacleFront  ← check_front_collision_sensor()
        sensedObstacleRight  ← check_right_collision_sensor()
        sensedBallColor      ← check_ball_color_Sensor()
        sensedBallPickUp     ← check_IR_detection()
```

## Planning 

Use sensing to make decisions.

```
robotPlanning
    Run finite state machines
    Example:
        fsmCollisionDetection()
        fsmMoveServoUpAndDown()
        fsmCapacitiveSensorSpeedControl()
```

Each FSM decides how to change the robot’s actions based on sensor input.

```
fsmCollisionDetection():
    if collisionInput == TRUE
        Set ActionDrive = STOP
        Set ActionCollisionIndicator = ON
    else
        Set ActionCollisionIndicator = OFF
        // let other FSMs decide driving
```

## Action

Implementing the decisions from planning to specific actions.

```
robotAction
    Apply planned actions to hardware
    Example:
        ActionCollision()
        ActionRobotDrive()
```

## Finite-State Machine Architecture

States:
- line_following
- ball_pick_up
- ball_drop_off
- canyon_navigation
- lander_laser

Enumerated types example
```c
enum {value list} state;

enum {line_following, ball_pick_up, etc} state;
if(state == line_following)
    actions()
    if(no_walls)
        state = canyon_navigation
```

## Random Tips

A quick reminder:

If you want to do digital input (read) from a pin that is shared with an analog input, you need to specifically tell the PIC that you want to do digital rather than analog input.

For example, pin 7 can be used as a digital input because it is bit 2 of port A, as seen on the pin diagram. Pin 7 can also be used as an analog input because it is listed as AN13 (analog channel 13) on the pin diagram.

To use pin 7 specifically as a digital input, you therefore need to do two things:

1. Use TRISA to specify that you want it to be an input
2. Use ANSA to specify that you want it to be a digital, rather than an analog, input
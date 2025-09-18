# Framework

The code is architectured into three different processes:

- Perception
- Planning
- Action

## Initialization

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

# Full Pseudo Code
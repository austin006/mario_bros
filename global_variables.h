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
int step = 0;
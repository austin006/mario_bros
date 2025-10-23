#include <xc.h>

void follow_line(void) 
{
    // config for 31 kHz oscillator
//    config_pwm_left_motor(39, 77);
//    config_pwm_right_motor(39, 77);

    // config for 500 kHz oscillator
    config_pwm_left_motor(600, 1249);
    config_pwm_right_motor(600, 1249);

    // Config for 8 MHz oscillator
//    config_pwm_left_motor(1250, 2499);
//    config_pwm_right_motor(1250, 2499);

    MOTOR_LEFT_DIR = FORWARD;
    MOTOR_RIGHT_DIR = FORWARD;
}
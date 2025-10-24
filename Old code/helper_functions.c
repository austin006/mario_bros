// List of helper functions
// File: helper_functions.c
// Fall 2025


#include <xc.h>

void config_pwm_right_motor(int duty, int period)
{
    // CONFIGURE PWM1 RIGHT STEPPER USING OC1 (on pin 14)
    OC1CON1 = 0;                // Clear all bits of OC1CON1
    OC1CON2 = 0;                // Clear all bits of OC1CON2
    OC1CON1bits.OCTSEL = 0b111; // System clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Self-synchronization
    OC1CON2bits.OCTRIG = 0;     // Synchronization mode
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    OC1RS = period;             // Set the PWM period
    OC1R = duty;                // Set the PWM duty cycle
}

void config_pwm_left_motor(int duty, int period)
{
    // CONFIGURE PWM2 LEFT STEPPER USING OC2 (on pin ??)
    OC2CON1 = 0;                // Clear all bits of OC2CON1
    OC2CON2 = 0;                // Clear all bits of OC1CON2
    OC2CON1bits.OCTSEL = 0b111; // System clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Self-synchronization
    OC2CON2bits.OCTRIG = 0;     // Synchronization mode
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    OC2RS = period;             // Set the PWM period
    OC2R = duty;                // Set the PWM duty cycle
}

void config_ad(void)
{
    // This function sets up configuration to read from multiple A/D channels in scan mode
    _ADON = 0;          // AD1CON1<15> -- Turn off A/D during config
    
    // Clear all A/D registers
    AD1CON1 = 0; 
    AD1CON2 = 0; 
    AD1CON3 = 0; 
    AD1CON5 = 0; 
    AD1CSSL = 0; 
    AD1CSSH = 0; 
    
    // AD1CON1 register
    _ADSIDL = 0;    // AD1CON1<13> -- A/D continues while in idle mode
    _MODE12 = 1;    // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;      // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;      // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;      // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;     // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;     // AD1CON2<13> -- Use VSS as negative ref voltage
    _BUFREGEN = 1;  // AD1CON2<11> -- Result appears in buffer
                    // location corresponding to channel, e.g., AN12
                    // results appear in ADC1BUF12
    _CSCNA = 1;     // AD1CON2<10> -- Scans inputs specified in AD1CSSx
                    // registers
    _SMPI = 2;    // AD1CON2<6:2> -- Results sent to buffer after n conversion
                    // For example, if you are sampling 4 channels, you
                    // should have _SMPI = 3;
    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register -- Change _SAMC and _ADCS based on your
    // selection of oscillator and postscaling
    _ADRC = 0;      // AD1CON3<15> -- Use system clock
    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
    
    // AD1CHS register
    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS

    // AD1CSSL register
    // SET THE BITS CORRESPONDING TO CHANNELS THAT YOU WANT TO SAMPLE
    AD1CSSL = 0x6010; // Read from pins 6,7,9 (B2, A2, A3)
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
}

void configureTimer(int time) {
    // Configure Timer1
    _TON = 1;           // Turn on timer
    _TCS = 0;           // Internal clock source
    _TCKPS = 0b11;      // Divide-by-256 prescaling

    // Reset Timer1 count to zero initially
    TMR1 = 0;
}
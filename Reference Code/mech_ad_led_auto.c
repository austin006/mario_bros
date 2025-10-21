// This program causes the LED to light up when an
// input voltage reaches a threshold.
//
// The A/D operates in automatic mode.
//
// Connections:
//      8:  Pot         (analog input -- AN14)
//      14: LED         (digital output -- RA6)
// File:    mech_ad_led_auto.c
// Author:  Mark Colton


#include <xc.h>

#pragma config FNOSC = FRCDIV   // 8 MHz FRC oscillator with postscaler
#pragma config OSCIOFNC = OFF   // Turn off clock output on pin 8
#pragma config SOSCSRC = DIG    // Turn off secondary oscillator on pins 9&10

void config_ad(void);

int main()
{    
    // Postscale oscillator
    _RCDIV = 0b011;             // Divide-by-8 postscaler
    
    // Configure pin 14 (RA6) as digital out for LED
    _TRISA6 = 0;
    _LATA6 = 0;
    
    // Configure pin 8 (RA3) as analog input for pot
    _TRISA3 = 1;
    _ANSA3 = 1;
       
    // Configure peripherals
    config_ad(); 

        
    while(1)
    {
        if (ADC1BUF14 >= 2048)
        {
            _LATA6 = 1;
        }
        else
        {
            _LATA6 = 0;
        }
    }
    
    return 0;
}

void config_ad(void)
{
    
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
    _CSCNA = 0;     // AD1CON2<10> -- Does not scan inputs specified in AD1CSSx
                    // registers
    _SMPI = 0;      // AD1CON2<6:2> -- Results sent to buffer after each
                    // conversion because there are is only one channel
                    // being converted
    _ALTS = 0;      // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register
    _ADRC = 0;      // AD1CON3<15> -- Use system clock
    _SAMC = 1;      // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0;      // AD1CON3<7:0> -- A/D period TAD = TCY
    
    // AD1CHS register
    _CH0NA = 0;     // AD1CHS<7:5> -- Measure voltages relative to VSS
    _CH0SA = 14;    // AD1CHS<4:0> -- Use AN2 (pin 4) as positive input
    
    _ADON = 1;      // AD1CON1<15> -- Turn on A/D
}
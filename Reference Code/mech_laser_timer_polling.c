// This program turns on and off a laser pointer module
// connected to pin 14 (RA6) by polling a timer.
//
// File:    mech_laser_timer_polling.c
// Author:  Mark Colton
// Date:    9/25/23

#include <xc.h>

// Select oscillator
#pragma config FNOSC = FRC      // 8 MHz FRC

#define PERIOD 31250            // 2-second period

// Main function
int main()
{   
    // Configure pin 14 (RA6)
    _TRISA6 = 0;        // Digital output
    
    // Configure Timer1
    _TON = 1;           // Turn on timer
    _TCS = 0;           // Internal clock source
    _TCKPS = 0b11;      // Divide-by-256 prescaling

    // Reset Timer1 count to zero initially
    TMR1 = 0;

    // Set pin 14 (RA6) initially high
    _LATA6 = 1;

    // Just loop continually and let the timer interrupt handle
    // everything in the background...
    while(1)
    {
        if (TMR1 >= PERIOD)
        {
            // Change state of pin 14 (RA6)
            _LATA6 = _LATA6 ^ 1;
            
            // Reset timer count
            TMR1 = 0;
        }
    }
   
    return 0;
}


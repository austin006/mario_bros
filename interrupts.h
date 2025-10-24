#include <xc.h> 

void __attribute__((interrupt, no_auto_psv))  _OC1Interrupt(void) {
        _OC1IF = 0;
        step++;
} 
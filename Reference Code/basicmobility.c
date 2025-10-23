/*
 * File:   basicmobility.c
 * Author: jmoulder
 *
 * Created on October 21, 2025, 4:40 PM
 */


#include "xc.h"

#pragma config FNOSC = LPFRC
#define dright _LATB12
#define dleft _LATA1
#define forward 1
#define backward 0

int step = 0;
int N = 0;

 void __attribute__((interrupt, no_auto_psv))  _OC1Interrupt(void) {
        _OC1IF = 0;
        step++;
          
    } 

int main(void) {
    
    
    _RCDIV = 0b000;
      // Configure PWM right stepper
    OC1CON1 = 0;                // Clear all bits of OC1CON1
    OC1CON2 = 0;                // Clear all bits of OC1CON2
    OC1CON1bits.OCTSEL = 0b111; // System clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Self-synchronization
    OC1CON2bits.OCTRIG = 0;     // Synchronization mode
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    // Configure PWM Left Stepper
    OC2CON1 = 0;                // Clear all bits of OC2CON1
    OC2CON2 = 0;                // Clear all bits of OC1CON2
    OC2CON1bits.OCTSEL = 0b111; // System clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Self-synchronization
    OC2CON2bits.OCTRIG = 0;     // Synchronization mode
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    // Configure direction pins
    _TRISA1 = 0;
    _ANSA1 = 0;
    _TRISB12 = 0;
    _ANSB12 = 0;
    // Enable interrupt
     _OC1IP = 4;
    _OC1IE = 1;
    _OC1IF = 0;
    
    
    
    // Go Forward
    N = 600;
    
    OC1RS = 1249;
    OC1R = OC1RS/2;
    OC2RS = 1249;
    OC2R = OC2RS/2;
    dright = forward;
    dleft = forward;
    step = 1;
    while(1){
        if (step >= N) {
            OC1RS = 0;
            OC1R = 0;
            OC2RS = 0;
            OC2R = 0;
            _OC1IE = 0;
            break;
            
        }
    
    }
    
  
    
    // Turn 90 Degrees
    OC1RS = 2449;
    OC1R = OC1RS/2;
    OC2RS = 2449;
    OC2R = OC2RS/2;
    N = 122;
    _OC1IE = 1;
    dright = backward;
    dleft = forward;
    step = 0;
    while(1){
        if (step >= N) {
            OC1R = 0;
            OC2R = 0;
            _OC1IE = 0;
            break;
        }
    }
    
    // Go forward
        N = 200;
    
    OC1RS = 1249;
    OC1R = OC1RS/2;
    OC2RS = 1249;
    OC2R = OC2RS/2;
    dright = forward;
    dleft = forward;
    _OC1IE = 1;
    step = 0;
    while(1){
        if (step >= N) {
            OC1R = 0;
            OC2R = 0;
            _OC1IE = 0;
            break;
        }
    
    }
    
    // Turn 180 Deg

    OC1RS = 2449;
    OC1R = OC1RS/2;
    OC2RS = 2449;
    OC2R = OC2RS/2;
    N = 243;
    dright = backward;
    dleft = forward;
    step = 0;
    _OC1IE = 1;
    while(1){
        if (step >= N) {
            OC1R = 0;
            OC2R = 0;
            break;
        }
    }
        
      // Go forward
        N = 200;
    
    OC1RS = 1249;
    OC1R = OC1RS/2;
    OC2RS = 1249;
    OC2R = OC2RS/2;
    dright = forward;
    dleft = forward;
    _OC1IE = 1;
    step = 0;
    while(1){
        if (step >= N) {
            OC1R = 0;
            OC2R = 0;
            break;
        }
    
    }   
    
    while (1){}
    
    
    return 0;
}
/*
 * File:   PulseOX.c
 * Author: Dhayalan balasubramanian
 *
 * Created on April 10, 2024, 9:26 AM
 */


#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "I2C.h"
#include "rgb_lcd_display.h"
#include "max30102.h"
#include "spo2_alg.h"
#include "button_switch.h"



// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL


//extern void delay_ms(unsigned int ms);
void pic24_init(void);   
// Initialization of necessary peripherals and sensors
void setup() {
    pic24_init();                  // Initialize PIC24 hardware configurations
    max30102_init();               // Initialize the MAX30102 sensor
    max30102_setup_spo2();         // Setup the sensor for SpO2 measurement
    init_I2C1();                    // Initialize I2C communication
    grovergb_init();               // Initialize the Grove LCD
    rgb_clr();                     // Clear the LCD display
    rgb_home();                    // Return cursor to the home position
    initButton();
}

void pic24_init() {
    CLKDIVbits.RCDIV = 0;  //clock to 16
    AD1PCFG = 0xffff; //all pins digital
    TRISAbits.TRISA0 = 0; 
}

int main(void) {
    setup();  // Perform all initial setup tasks

    while(1) {
            rgb_clr();
            max30102_process_signals();

    }
    return 0;
}

#include "I2C.h"
/*
 * Date: 4/24/2024
 * Name(s): Cole Jaeger, Dhayalan Balasubarmanian, Luke Tschepen, Nick
 * Project Name: Blood Oxygen Sensor
 * Program Description: contains all the I2C functions that 
 * we used for the different components of the project. 
 * We used I2C1 for the LCD and I2C for the sensor. 
 * This file also includes the functions for reading and writing for the registers using I2C. 
 * 
 */

/*
function that initializes the I2C parameters. 
*/
void init_I2C1(void){
    I2C1BRG = 37; //set to a clock frequency of 400 kHz section 16.3 PIC24 D.S.
    IFS1bits.MI2C1IF = 0; //Clear Interrupt Flag 
    I2C1CONbits.I2CEN = 1; //Enable I2C Mode
}

//function that initiates the start condition for I2C1 communication.
void I2C1_start() {
    I2C1CONbits.SEN = 1;   // Initiate start condition
    while (I2C1CONbits.SEN);
};

//function that initiates the stop condition for I2C1 communication.
void I2C1_stop() {
    I2C1CONbits.PEN = 1;   // Initiate stop condition
    while (I2C1CONbits.PEN);
}

// Function to write data over I2C1
//data - given data to be written
void I2C1_write(uint8_t data) {
    I2C1TRN = data;        //  I2C transimit reg
    while (I2C1STATbits.TRSTAT); // Wait for transmission to complete
}

//function that initiates the start condition for I2C communication.
void I2C_start() {
    I2C2CONbits.SEN = 1;   // Initiate start condition
    while (I2C2CONbits.SEN);
};

//function that initiates the stop condition for I2C communication.
void I2C_stop() {
    I2C2CONbits.PEN = 1;   // Initiate stop condition
    while (I2C2CONbits.PEN);
};
// Function to write data over I2C
void I2C_write(uint8_t data) {
    I2C2TRN = data;        //  I2C transimit reg
    while (I2C2STATbits.TRSTAT); // Wait for transmission to complete
}

// Function to read data over I2C
//enables receiving mode and returns received element
uint32_t I2C_read() {
    I2C2RCV = 0;
    I2C2CONbits.RCEN = 1; // Enable receive mode
    while (I2C2CONbits.RCEN == 1);
    return I2C2RCV;        // I2C receive re
}

/*
 * function that initiates the repeated start condition and 
 * waits for the start condition to complete. 
 */
void I2C_repeated_start() {
    I2C2CONbits.RSEN = 1;  // Initiate repeated start condition
    while (I2C2CONbits.RSEN == 1);
    //while (I2C2CONbits.RSEN); // Wait for repeated start condition to complete
}

/*
 * function that sends acknowledgment over I2C communication.
 */
void I2C_ACK() {
    I2C2CONbits.ACKDT = 0;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);
}

/*
 * function that sends non acknowledgment over I2C communication.
 */
void I2C_NACK() {
    I2C2CONbits.ACKDT = 1;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);
}

/*
 * Reads a single byte of data from a specific register address over the I2C. 
 * uses the same process as reading the PartID
 * 
 * reg - register address to read from 
 */
uint8_t I2C_read_with_params(uint8_t reg) {
    I2C_start();

    // Send device address with write bit
    I2C_write(MAX30102_ADDRESS_WRITE);

    // Send register address
    I2C_write(reg);

    // Restart I2C communication
    I2C_repeated_start();

    // Send device address with read bit
    I2C_write(MAX30102_ADDRESS_READ);

    // Read received data
    uint8_t data = I2C_read();
    
    I2C2CONbits.ACKDT = 1;
        
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);

    // Stop I2C communication
    I2C_stop();

    return data;
}

/*
 * writes a single byte of data from a specific register address over the I2C. 
 * writes to the register with the value to put in
 * 
 * reg - address to read to 
 * 
 * val - value to put into address to match the configurations
 */
void I2C_write_with_params(uint8_t reg, uint8_t val) {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(reg); 

    I2C_write(val);       

    I2C_stop();
}

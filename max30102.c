#include "max30102.h"
#include <stdio.h>
#include "I2C.h"
#include "rgb_lcd_display.h"
#include "spo2_alg.h"
#include "button_switch.h"

/*
 * Date: 4/24/2024
 * Name(s): Cole Jaeger, Dhayalan Balasubarmanian, Luke Tschepen, Nick
 * Project Name: Blood Oxygen Sensor
 * Program Description:This file is the library for the MAX30102 sensor. This stores all the registers needed for 
 * configuring and using the sensor and also the functions that use I2C protocol to configure the sensor correctly. 
 * This file also has functions that read data from the buffer and use an Arduino algorithm to convert those readings into
 * proper SPO2 and heart rate readings which are then put into a moving average for more accurate data. 
 * 
 * 
 */

/*

*/
volatile unsigned long int overflow = 0;

static const uint8_t MAX30105_INTSTAT1 =		0x00;
static const uint8_t MAX30105_INTSTAT2 =		0x01;
static const uint8_t MAX30105_INTENABLE1 =		0x02;
static const uint8_t MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
//static const uint8_t MAX30105_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

//static const uint8_t MAX30105_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

//tatic const uint8_t MAX30105_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 		0x80;
static const uint8_t MAX30105_WAKEUP = 			0x00;

static const uint8_t MAX30105_RESET_MASK = 		0xBF;
static const uint8_t MAX30105_RESET = 			0x40;

static const uint8_t MAX30105_MODE_MASK = 		0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

volatile int readIdxRED = 0;
volatile int writeIdxRED = 0;
volatile int readIdxIR = 0;
volatile int writeIdxIR = 0;
volatile int numElemsInBuffRED = 0;
volatile int numElemsInBuffIR = 0;
volatile uint32_t RED_Buffer[BUFFER_SIZE];
volatile uint32_t IR_Buffer[BUFFER_SIZE];

int32_t spo2 = 0;
int8_t validSpo2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;
uint8_t partID = 0;
uint32_t heartRateAvg;
uint32_t spo2Avg;

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t I2C_read_with_params(uint8_t reg);
void I2C_write_with_params(uint8_t reg, uint8_t val);
void setLEDMode(uint8_t mode);


/*
 * put function which puts data in the RED LED buffer. 
 * 
 * data - long int of data to put in the buffer. 
 */
void max30102_RED_putBuff(long int data){ 
    if (numElemsInBuffRED < BUFFER_SIZE){
               RED_Buffer[writeIdxRED++] = data;
               writeIdxRED %= BUFFER_SIZE;
               ++numElemsInBuffRED;
    } 
}

/*
 * function that gets a element from the RED LED buffer. 
 */
long int max30102_RED_getBuff() {
        long int x;
        x = RED_Buffer[readIdxRED++];
        readIdxRED %= BUFFER_SIZE;
        --numElemsInBuffRED;
        return x;
    //return RED_Buffer[0];
}

/*
 * put function which puts data in the IR LED buffer. 
 * 
 * data - long int of data to put in the buffer. 
 */
void max30102_IR_putBuff(long int data){
    if (numElemsInBuffRED < BUFFER_SIZE){
               IR_Buffer[writeIdxIR++] = data;
               writeIdxIR %= BUFFER_SIZE;
               ++numElemsInBuffIR;
    } 
}

/*
 * function that gets a element from the IR LED buffer. 
 */
long int max30102_IR_getBuff() {
        long int x;
        x = IR_Buffer[readIdxIR++];
        readIdxIR %= BUFFER_SIZE;
        --numElemsInBuffIR;
        return x;
    //return IR_Buffer[0];
}

//Initializes the MAX30102 sensor by configuring its settings and registers.
void max30102_init(void) {
    I2C2CONbits.I2CEN = 0;
    
    I2C2BRG = 0x25; //set to a clock freq of 400kHz and 16MHz Fcy/ BAUD RATE
//    TRISBbits.TRISB6 = 1;  // SDA pin as input
//    TRISBbits.TRISB7 = 1;  // SCL pin as input
    I2C2CONbits.I2CEN = 1; //Enable I2C mode
    //I2C2CONbits.SMODE = 0;

    IFS3bits.MI2C2IF = 0; // clr Int flag
}


/*
 * timer1 interrupt to take care of the overflow
 */
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void) {
    _T1IF = 0;
    overflow++;
}


/*
 * setup the timer1 with the correct parameters. 
 */
void timer1_setup(void) {
    T1CONbits.TON = 0;
    PR1 = 1999; 
    T1CONbits.TCKPS = 1; 
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

/*
 * Writes the configuration for the SpO2 (blood oxygen saturation) sensor mode.
 * USes the process of I2C start, and write to the write address of the I2C and writes to the correct addresses. 
 * Configs in datasheet pg. 18
 */
void max30102_write_config_SP02() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x0A); // REG ADDR of the SPO2
    I2C_write(0x27);   //00100111    00100001
    //0, 01 - ADC range, 001 - SPO2 sample rate control, 11 - LED pulse width control
    
    I2C_stop();
}

/*
 * Writes the configuration for the FIFO (First In, First Out) buffer of the MAX30102 sensor.
 * USes the process of I2C start, and write to the write address of the I2C and writes to the correct addresses of the FIFO
 * Configs in datasheet pg. 17
 */
void max30102_write_config_FIFO() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x08);  //write to the FIFO REG ADDR

    I2C_write(0x40);     //0b10111111   
    //0b1000000 100 - NO. OF SAMPLES AVERAGED PER FIFO SAMPLE, 0 - FIFO_ROLLOVER_EN, 000 - FIFO_A_FULL

    I2C_stop();
}

/* Writes the configuration for the operating mode of the MAX30102 sensor.
 * Uses the process of I2C start, and write to the write address of the I2C and writes to the correct addresses of the MODE configs
 * Configs in datasheet pg. 17
 */
 void max30102_write_config_MODE() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x09);  //write to the MODE REG ADDR

    I2C_write(0X03);       //011 for SpO2 mode Red and IR

    I2C_stop();
}

 /* function which writes the configuration to reset the operating mode of the MAX30102 sensor.
 * Uses the process of I2C start, and write to the write address of the I2C and writes to the correct addresses of the MODE configs
  * Configs in datasheet pg. 17
 */
 void max30102_write_config_RESET_MODE() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x09); 

    I2C_write(0x40);     //0b1000000 1 - SHDN, 000, 000 - mode

    I2C_stop();
}

/*
  * function which reads the Part ID register of the MAX30102 sensor to verify its identity.
  * starts by sending device address and write mode, writes to the PART_ID, sends repeated start and get the partID with I2C read. 
  * sends ACK and then stops I2C read. 
  */
uint32_t max30102_read_partID() {
        I2C_start();  
        I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
        

        I2C_write(PART_ID); // Send address of PART_ID
        

        I2C_repeated_start(); // Repeated start condition unnecessary
        
        I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
        

        partID = I2C_read(); // Read PART_ID
        
        I2C2CONbits.ACKDT = 1;
        
        I2C2CONbits.ACKEN = 1;
        while (I2C2CONbits.ACKEN == 1);
        //LATAbits.LATA0 = 1;
        I2C_stop();
        
        return partID;

}

/*
 * Modifies specific bits of a register by applying a mask and setting new values.
 * 
 * reg - register to read from
 * mask - mask address
 * val - bits to change to
 */
void bitMask(uint8_t reg, uint8_t mask, uint8_t val) {
    // Grab current register context
    uint8_t originalContents = I2C_read_with_params(reg);

    // Clear bits not protected by mask and set bits according to val
    originalContents = (originalContents & mask) | (val & ~mask);

    // Write back the modified contents to the register
    I2C_write_with_params(reg, originalContents);
}


// function which sets which LEDs are used for sampling -- Red only, RED+IR only, or custom.
// mode - the mode value to set mode as
void setLEDMode(uint8_t mode) {
    // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    // See datasheet, page 19
    bitMask(MAX30102_MODECONFIG, MAX30105_MODE_MASK, mode);
}

//function which sets the pulse width for LED operation.
//pulseWidth - pulsewidth value to set to
void setPulseWidth(uint8_t pulseWidth) {
    bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

//function which sets the amplitude of the RED LED 
//amplitude - amplitude value to set to for RED LED
void setPulseAmplitudeRed(uint8_t amplitude) {
    I2C_write_with_params(MAX30105_LED1_PULSEAMP, amplitude);
}

//function which sets the amplitude of the proximity sensor LED 
//amplitude - amplitude proximity value to set to 
void setPulseAmplitudeProximity(uint8_t amplitude){
    I2C_write_with_params(MAX30105_LED_PROX_AMP, amplitude);
}

//function which sets the amplitude of the IR LED 
//amplitude - amplitude value to set to for IR LED
void setPulseAmplitudeIR(uint8_t amplitude) {
  I2C_write_with_params(MAX30105_LED2_PULSEAMP, amplitude);
}

//function that wakes up the MAX30102 sensor from a low-power state.
void max30102_wakeUp(void) {
  bitMask(MAX30102_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

//function that enables a time slot for a specific device 
//slot number - slot to set as
//device - device to be enabled with slot
void max30102_enableSlot(uint8_t slotNumber, uint8_t device) {
    switch (slotNumber) {
        case 1:
            bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
            break;
        case 2:
            bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
            break;
        case 3:
            bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
            break;
        case 4:
            bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
            break;
        default:
            break;
    }
}

//Puts the MAX30102 sensor into a low-power shutdown mode.
void max30102_shutdown(void) {
    bitMask(MAX30102_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

//function which sets the Analog-to-Digital Converter (ADC) range for sensor measurements.
//adcRange - adcRange value to set to for the ADC
void max30102_setADC(uint8_t adcRange) {
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

//function which sets the sample rate range for sensor measurements.
//sampleRate - sample rate value to set 
void max30102_setSampleRate(uint8_t sampleRate) {
  // sampleRate: MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

//function that sets the number of samples for FIFO buffer averaging.
//numSamples - samples to set for the FIFO average
void max30102_setFIFOAvg(uint8_t numSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numSamples);
}

//function which clears the FIFO buffer of the MAX30102 sensor.
void max30102_clearFIFO(void) {
  I2C_write_with_params(MAX30105_FIFOWRITEPTR, 0);
  I2C_write_with_params(MAX30105_FIFOOVERFLOW, 0);
  I2C_write_with_params(MAX30105_FIFOREADPTR, 0);
}

//Enables FIFO buffer rollover to prevent overflow.
void max30102_enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

//Disables FIFO buffer rollover.
void max30102_disableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}


//Sets the number of samples to trigger the FIFO almost full interrupt.
//numberOfSamples  - number of samples for the FIFO
void max30102_setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

//Sets the proximity detection threshold.
//threshMSB  - signifies only the 8 most significant-bits of the ADC count.
void max30102_setProximityThreshold(uint8_t threshMSB) {
    // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // See datasheet, page 24.
    I2C_write_with_params(MAX30105_PROXINTTHRESH, threshMSB);
}

//Read the FIFO Write Pointer
//returns the write pointer frfom FIFO
uint8_t max30102_getWritePointer(void) {
  return (I2C_read_with_params(MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
//returns the read pointer from FIFO
uint8_t max30102_getReadPointer(void) {
  return (I2C_read_with_params(MAX30105_FIFOREADPTR));
}

//Reads the temperature data from the MAX30102 sensor.
void max30102_readTemp(){
    I2C_write_with_params(MAX30105_DIETEMPCONFIG, 0X01);
    overflow = 0;
    while(overflow < 100) {
        uint8_t temp_Response = I2C_read_with_params(MAX30105_INTSTAT2);
        if ((temp_Response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
        delay_ms(1);
        
    }
    
    int8_t tempInt = I2C_read_with_params(MAX30105_DIETEMPINT);
    uint8_t tempFrac = I2C_read_with_params(MAX30105_DIETEMPFRAC);
    uint8_t result = (float)tempInt + ((float)tempFrac * 0.0625);
}

//interrupt config for interrupt1
//returns the interrupt status1
uint8_t max30102_int1(){
    return (I2C_read_with_params(MAX30105_INTSTAT1));
}

//interrupt config for interrupt2
//returns the interrupt status2
uint8_t max30102_int2(){
    return (I2C_read_with_params(MAX30105_INTSTAT2));
}

//configs and setups a software reset of the MAX30102 sensor.
void softReset(void) {
    bitMask(MAX30102_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

    TMR1 = 0; // Reset Timer 1
    while (TMR1 < 100) { // Timeout after 100 ms
        uint8_t response = I2C_read_with_params(MAX30102_MODECONFIG);
        if ((response & MAX30105_RESET) == 0) break; // We're done!

        delay_ms(1); // Let's not overburden the I2C bus, use a blocking delay
    }
}

//reads from the sensor
void getRead(){
    //I2C_read_multiple_bytes_params(MAX30105_FIFODATA, 1);
    //getNewData(MAX30105_FIFODATA, 3);
      getNewData2();
//    I2C_start();
//    I2C_write(MAX30102_ADDRESS_WRITE);
//    I2C_write_with_params(MAX30105_FIFOREADPTR, MAX30105_FIFOREADPTR);
//    I2C_stop();      
}

/*
 * follows the datasheet algorithm to get the read in the data from the sensor.  
 */
void getNewData2(void) {
    uint8_t readPointer = max30102_getReadPointer();
    uint8_t writePointer = max30102_getWritePointer();
    uint8_t numSamples = (writePointer - readPointer) & 0x1F; // Assuming 32-point buffer, & 0x1F handles wrap-around

    if (numSamples == 0) {
        //setCursor(0, 0);
        //lcd_printStr("!NO DATA!");
        return; // Exit if no data
    }

    int32_t bytesToRead = numSamples * 6; // 6 bytes per sample (3 bytes RED, 3 bytes IR)
   
    I2C_start();
    I2C_write(MAX30102_ADDRESS_WRITE);
    I2C_write(MAX30105_FIFODATA); // Address of FIFO data register
    I2C_repeated_start();
    I2C_write(MAX30102_ADDRESS_READ);
    if (numSamples < 0) {
        numSamples += 32;
    }
    while (bytesToRead > 0) {
        uint32_t red_sample = 0;
        uint32_t ir_sample = 0;

        // Read RED sample
        red_sample |= (uint32_t)I2C_read() << 16; // MSB
        I2C_ACK();
        red_sample |= (uint32_t)I2C_read() << 8; // Middle byte
        I2C_ACK();
        red_sample |= I2C_read(); // LSB
        I2C_ACK(); // ACK after each byte except the last byte of the last sample
        max30102_RED_putBuff(red_sample & 0x3FFFF);

        // Read IR sample
        ir_sample |= (uint32_t)I2C_read() << 16; // MSB
        I2C_ACK();
        ir_sample |= (uint32_t)I2C_read() << 8; // Middle byte
        I2C_ACK();
        ir_sample |= I2C_read(); // LSB
        bytesToRead -= 6; // Reduce the byte counter by 6 (3 for RED, 3 for IR)
        if (bytesToRead > 0) {
            I2C_ACK();
        } else {
            I2C_NACK(); // NACK after last byte of the last sample
        }
        max30102_IR_putBuff(ir_sample & 0x3FFFF);
    }

    I2C_stop(); // Stop I2C communication after reading all data
}

/*
 * setup algorithm for the spo2
 * sets all th correct parameters all seperated by a delay. 
 */
void max30102_setup_spo2(){
    delay_ms(100);
    softReset();
    delay_ms(100);
    max30102_setFIFOAvg(MAX30105_SAMPLEAVG_4);//8
    delay_ms(100);
    max30102_enableFIFORollover();
    delay_ms(100);
    setLEDMode(MAX30105_MODE_REDIRONLY);
    delay_ms(100);
    max30102_setADC(MAX30105_ADCRANGE_4096);//4096
    delay_ms(100);
    max30102_setSampleRate(MAX30105_SAMPLERATE_100); //800
    delay_ms(100);
    setPulseWidth(MAX30105_PULSEWIDTH_411);
    delay_ms(100);
    setPulseAmplitudeRed(0x1F);
    delay_ms(100);
    setPulseAmplitudeIR(0x1F);
    delay_ms(100);
    setPulseAmplitudeProximity(0x1F);
    delay_ms(100);
    max30102_enableSlot(1, SLOT_RED_LED);
    delay_ms(100);
    max30102_enableSlot(2, SLOT_IR_LED);
    delay_ms(100);
    max30102_clearFIFO(); 
}

// Function to calculate moving average
//buffer - given buffer to look into
//size - size of the buffer
//returns the average of the given buffer
uint32_t calculateMovingAverage(uint32_t *buffer, int size) {
    uint32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}


    uint32_t irBuffer[100];  // Buffer for infrared sensor data
    uint32_t redBuffer[100]; // Buffer for red sensor data
    uint32_t heartRateBuffer[MOVING_AVG_SIZE_HR] = {0}; // Buffer for heart rate moving average
    uint32_t spo2Buffer[MOVING_AVG_SIZE_SPO2] = {0}; // Buffer for SpO2 moving average


    char adStr[32];  // Buffer for display strings
    char adStr2[32]; // Buffer for display strings

/*
gets calculates data from the buffers and stores them in the buffer, then calculates moving average of the buffer
*/
void max30102_process_signals(){        
        getRead();
        // Read 100 samples into the buffers
        for (int i = 0; i < 100; i++) {
                     irBuffer[i] = max30102_IR_getBuff();    // Get one IR sample
                    redBuffer[i] =  max30102_RED_getBuff();  // Get one Red sample
        }

        // Process the collected data
        maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSpo2, &heartRate, &validHeartRate);
        if (spo2 > 85) {
             // Update heart rate buffer with latest value
            for (int i = 0; i < MOVING_AVG_SIZE_HR - 1; i++) {
                heartRateBuffer[i] = heartRateBuffer[i + 1];
            }
            if (heartRate > 0){
                heartRateBuffer[MOVING_AVG_SIZE_HR - 1] = heartRate;

            }

            // Update SpO2 buffer with latest value
            for (int i = 0; i < MOVING_AVG_SIZE_SPO2 - 1; i++) {
                spo2Buffer[i] = spo2Buffer[i + 1];
            }
            if (spo2 > 0){
                spo2Buffer[MOVING_AVG_SIZE_SPO2 - 1] = spo2;

            }

            // Calculate moving average of heart rate
            heartRateAvg = calculateMovingAverage(heartRateBuffer, MOVING_AVG_SIZE_HR);
            spo2Avg = calculateMovingAverage(spo2Buffer, MOVING_AVG_SIZE_SPO2);

            max30102_setup_display();
//            setCursor(0,0);
//            sprintf(adStr, "SPO2: %d %%", (int)spo2Avg);
//            printStr(adStr);
//            setCursor(0,1);
//            printStr("Please hold for 15 seconds");
//            delay_ms(500);

        }
        else {
            
            max30102_setup_default();
        }
}

// setups up the beginning string of the display
void max30102_setup_default(void) {
        //setRGB();
        setCursor(0,0);
        printStr("No Valid Data");
        setCursor(0,1); 
        printStr("Press 4 SPO2/HR"); //wed wanna scroll this.
        delay_ms(500);
}
//setups the displaying for the data on the display
void max30102_setup_display(void) {
    rgb_clr();            // Clear the LCD display for fresh data
    setCursor(0,0);
           
    if(getButtonState()){
            setColorRed();
            sprintf(adStr2, "Heart: %d BPM", (int)heartRateAvg);
            printStr(adStr2);
            setCursor(0,1);
            printStr("Press for SPO2");

    } else {
            setColorWhite();
            sprintf(adStr, "SPO2: %d %%", (int)spo2Avg);
            printStr(adStr);
            setCursor(0,1);
            printStr("hold for 15s");
    }

    delay_ms(500);  // Delay for stability and readability
}


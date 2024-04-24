


#ifndef MAX30102_H
#define MAX30102_H

#include <xc.h> // include processor files - each processor file is guarded.


// Define I2C communication parameters
#define MAX30102_ADDRESS_WRITE 0xAE //writing
#define MAX30102_ADDRESS_READ  0xAF //reading


// Define register addresses
#define FIFO_WR_PTR    0x04
#define FIFO_RD_PTR    0x06
#define FIFO_OVERFLOW  0x05
#define FIFO_DATA      0x07
#define FIFO_DEPTH     32
#define PART_ID        0xFF
#define BUFFER_SIZE 32
#define MOVING_AVG_SIZE_SPO2 10 // Adjust the moving average size as needed
#define MOVING_AVG_SIZE_HR 40

extern int32_t spo2;
extern int8_t validSpo2;
extern int32_t heartRate;
extern int8_t validHeartRate;
extern uint8_t partID;
extern uint32_t heartRateAvg;
extern uint32_t spo2Avg;

#ifdef __cplusplus
extern "C" {
#endif

void max30102_RED_putBuff(long int data);
long int max30102_RED_getBuff();
void max30102_IR_putBuff(long int data);
long int max30102_IR_getBuff();
void max30102_setup_spo2(void);
void max30102_init(void);
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void);
void timer1_setup(void);
void max30102_write_config_SP02();
void max30102_write_config_FIFO();
void max30102_write_config_MODE();
void max30102_write_config_RESET_MODE();
uint32_t max30102_read_partID();
void bitMask(uint8_t reg, uint8_t mask, uint8_t val);
void setLEDMode(uint8_t mode);
void setPulseWidth(uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t amplitude);
void setPulseAmplitudeProximity(uint8_t amplitude);
void setPulseAmplitudeIR(uint8_t amplitude);
void max30102_wakeUp(void);
void max30102_enableSlot(uint8_t slotNumber, uint8_t device);
void max30102_shutdown(void);
void max30102_setADC(uint8_t adcRange);
void max30102_setSampleRate(uint8_t sampleRate);
void max30102_setFIFOAvg(uint8_t numSamples);
void max30102_clearFIFO(void);
void max30102_enableFIFORollover(void);
void max30102_disableFIFORollover(void);
void max30102_setFIFOAlmostFull(uint8_t numberOfSamples);
void max30102_setProximityThreshold(uint8_t threshMSB);
uint8_t max30102_getWritePointer(void);
uint8_t max30102_getReadPointer(void);
void max30102_readTemp();
uint8_t max30102_int1();
uint8_t max30102_int2();
void softReset(void);
void getRead();
void getNewData2(void);
uint32_t calculateMovingAverage(uint32_t *buffer, int size);
void max30102_process_signals();
void max30102_setup_default();
void max30102_setup_display();


 

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_H */
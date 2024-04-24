/*
 * Date: 4/9/2024
 * Name(s): Cole Jaeger, Dhayalan Balasubarmanian, Luke Tschepen, Nicholas Kang
 * Project Name: Blood Oxygen Sensor
 * Program Description: This program initializes the Grove RGB/LCD display
 * to be compatible with the PIC24 using I2C communication. It contains various
 * functions to manipulate and display different text characters and colors on
 * the display.
 * References/Code Citations: Arduino Grove RGB file, JHD data sheet
 * 
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

void delay_ms(unsigned int ms){
    //uses c-ASM code to delay 1 millisecond
    //ms: number of milliseconds to delay
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

void rgb_cmd(char Package) {
    // I2C communication protocol betweem the PIC24 and Grove LCD
    // Used to send a command with data to the LCD 
    // Package: an 8-bit command
    
    I2C1_start();
    
    I2C1_write(LCD_SLAVE_ADDR);
    
    I2C1_write(COMMANDBIT);
    
    I2C1_write(Package);
    
    I2C1_stop();
    
}

void setReg(unsigned char reg, unsigned char dat) {
    //I2C protocol used to set the LED registers for the RGB display
    //reg: desired LED register
    //data: binary number representing a color from 0-255
    I2C1_start();
    I2C1_write(RGB_SLAVE_ADDR);
    I2C1_write(reg);
    I2C1_write(dat);
    I2C1_stop();
}

void setRGB(unsigned char r, unsigned char g, unsigned char b){
    //calls the setReg function for each RGB color
    //r: red value from 0-255
    //g: green value from 0-255
    //b: blue value from 0-255
    setReg(0x04, r);
    setReg(0x03, g);
    setReg(0x02, b);
}

void rgb_clr(){
    //clears the text on the display
    rgb_cmd(DISPLAY_CLR);
}

void rgb_home() {
    //returns the cursor to the initial position (0,0)
    rgb_cmd(LCD_RETURNHOME);
    delay_ms(2000);
}

void noDisplay() {
    //turns the display off
    rgb_cmd(DISPLAY_CONTROL);
}

void display() {
    //turns the display on
    rgb_cmd(DISPLAY_CONTROL | LCD_DISPLAYON);
}

void blinkLED(void){
    //flashes the specified LED registers
    setReg(0x07, 0x17);
    setReg(0x06, 0x7F);
}

void noBlinkLED(){
    //stops the flashing of the LED registers
    setReg(0x07, 0x00);
    setReg(0x06, 0xFF);
}

void setColorAll() {
    //turns all colors off
    setRGB(0,0,0);
}

void setColorWhite() {
    //turns the screen white
    setRGB(255, 255, 255);
}

void setColorRed() {
    //turns the screen red
    setRGB(255, 0, 0);
}

void setCursor(uint8_t x, uint8_t y){
    //sets the cursor to the desired position on the screen
    //x: column (0-15)
    //y: row (0-1)
    rgb_cmd(0x0 | 0x80 | (0x40 * y + x));
}

void grovergb_init(void){
    //initialization from Grove-LCD/RGB data-sheet and Arduino reference code
    delay_ms(50);//wait > 30ms
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(DISPLAY_CONTROL | LCD_DISPLAYON); //display ON/OFF Control 00001DCB D = display off/on (0/1) C = cursor off/on (0/1) B = blink off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_clr(); //display clear 00000001
    
    delay_ms(2); //wait > 1.53ms
    
    rgb_cmd(ENTRY_MODE_SET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT); //entry mode set 000001IS I = decrement/increment mode (0/1) S = entire shift off/on (0/1)
 
    delay_ms(1); //doesn't specify but why not //end initialization
    
    setReg(REG_MODE1, 0);
    
    setReg(REG_OUTPUT, 0xFF);
    
    setReg(REG_MODE2, 0x20);
    
    setColorWhite();
}

void printStr(const char s[]){
    //prints a string to the display
    //s: an array of characters representing a string
    int len = strlen(s);
    
    I2C1_start();
    I2C1_write(LCD_SLAVE_ADDR);
    
    for(int i = 0; i < len; i++ ){
       
        I2C1_write(COMMANDBIT | WRITEBIT);
        I2C1_write(s[i]); // 8-bits consisting of the data byte
    }
    I2C1_write(WRITEBIT);

    I2C1_stop();
    
}

void printChar(char myChar){ 
    //prints a character to the display
    //myChar: a character
    I2C1_start();
    I2C1_write(LCD_SLAVE_ADDR);
    I2C1_write(WRITEBIT);
    I2C1_write(myChar);
    I2C1_stop();
}

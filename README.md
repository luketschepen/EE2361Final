[https://github.com/luketschepen/EE2361Final/tree/main 

INTRO- Description of the Library and its overall functions
Members: Dhayalan Balasubramanian, Luke Tschepen, Nicholas Kang, Cole Jaeger.

The Blood Oxygen Sensor project integrates the MAX30102 sensor for pulse oximetry and heart-rate monitoring, alongside the Grove RGB LCD module for versatile display feedback. Physical input is managed by the button_switch.c file, facilitating interaction with a button switch for toggling display and capturing button events. Crucially, the algorithm.c file computes heart rate and SpO2 levels using peak detection techniques and precise algorithms on sensor data. The I2C.h file handles essential I2C communication for interfacing with the sensor and other peripherals.

Separate Descriptions
MAX 30102 Description:
The code implements functionalities for interfacing with the MAX30102 sensor, which is a pulse oximeter and heart-rate sensor. It includes functions for initializing the sensor, configuring its settings, reading sensor data, processing the signals, and displaying the results.

The Grove RGB LCD module is a multifunctional display that integrates an RGB backlight and a character LCD. It is designed to provide a versatile display solution for various electronic projects. This document outlines the functionality and usage of the Grove RGB LCD module within the context of the Blood Oxygen Sensor project.

The button_switch.c file provides functionality for interfacing with a physical button switch in the Blood Oxygen Sensor project. It includes functions for initializing the button, detecting button presses, and determining the state of the button. The button switch is connected to RB7 (Pin 6) of the microcontroller, and its usage involves capturing button press events and toggling the display of heart rate information

The algorithm.c file is a crucial component of the Blood Oxygen Sensor project, dedicated to computing heart rate and SpO2 levels. It implements peak detection techniques on data from IR and red sensors to identify PPG signal peaks for heart rate determination. Additionally, it employs a precise algorithm to derive SpO2 values from the ratio of AC to DC components in the IR and red signals.

The I2C.h file contains functions for initializing and managing I2C communication on PIC24 microcontrollers. It includes functions for initializing I2C parameters, initiating start and stop conditions, writing and reading data over I2C, handling acknowledgments, and performing repeated start conditions.



Hardware description - what device(s), part numbers, links, etc.
SNAP: MP LAB SNAP PG164100
https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/UserGuides/50002787C.pdf 


PIC24: PIC24FJ64GA002
https://ww1.microchip.com/downloads/en/DeviceDoc/39881e.pdf 

Sensor: MAX30102
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf 

RGB LCD: Grove-LCD RGB Backlight JHD1313
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf 


Button: Square Tactile Pushbutton Switch SPST Momentary -ELSW-TPB-SPST
https://www.ckswitches.com/media/1462/pts125.pdf 

Full documentation- description - what device(s), part numbers, links, etc.

MAX 30102 Description:
The code implements functionalities for interfacing with the MAX30102 sensor, which is a pulse oximeter and heart-rate sensor. It includes functions for initializing the sensor, configuring its settings, reading sensor data, processing the signals, and displaying the results.

Devices and Parts:
SNAP
PIC24
MAX30102 Sensor
RGB LCD display
Button switch

Links to datasheets and part numbers:
SNAP: MP LAB SNAP PG164100
https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/UserGuides/50002787C.pdf 


PIC24: PIC24FJ64GA002
https://ww1.microchip.com/downloads/en/DeviceDoc/39881e.pdf 

Sensor: MAX30102
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf 

RGB LCD: Grove-LCD RGB Backlight JHD1313
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf 


Button: Square Tactile Pushbutton Switch SPST Momentary -ELSW-TPB-SPST
https://www.ckswitches.com/media/1462/pts125.pdf

Public Functions:
Initialization and Configuration:
max30102_init(void): Initializes the MAX30102 sensor.
max30102_setup_spo2(): Configures the MAX30102 sensor for SpO2 measurement.
max30102_setup_default(void): Sets up the MAX30102 sensor to the default state.
max30102_setup_display(void): Configures the display for showing SpO2 and heart rate data.

Data Acquisition and Processing:
max30102_process_signals(): Processes the sensor signals to calculate SpO2 and heart rate.
max30102_readTemp(): Reads the die temperature from the sensor.
calculateMovingAverage(uint32_t *buffer, int size): Calculates the moving average of the data buffer.

Buffer Management:
max30102_RED_putBuff(long int data): Inserts data into the Red LED buffer.
max30102_IR_putBuff(long int data): Inserts data into the Infrared LED buffer.
max30102_RED_getBuff(): Retrieves data from the Red LED buffer.
max30102_IR_getBuff(): Retrieves data from the Infrared LED buffer.

Control and Configuration:
Functions to configure various parameters such as LED mode, ADC range, sample rate, pulse width, FIFO settings, etc.


Utility:
max30102_shutdown(void): Shuts down the sensor.
softReset(void): Performs a soft reset of the sensor.

Arguments and Outputs:
Many functions take parameters to configure specific settings on the sensor, such as LED mode, sample rate, pulse width, etc.
The functions for inserting and retrieving data from the buffers (max30102_RED_putBuff, max30102_IR_putBuff, max30102_RED_getBuff, max30102_IR_getBuff) accept and return data values.
Functions for processing signals (max30102_process_signals()) calculate SpO2 and heart rate and store the results in global variables.
Display functions (max30102_setup_default, max30102_setup_display) output data to an RGB LCD display.




Grove RGB LCD Documentation

Description:

The Grove RGB LCD module is a multifunctional display that integrates an RGB backlight and a character LCD. It is designed to provide a versatile display solution for various electronic projects. This document outlines the functionality and usage of the Grove RGB LCD module within the context of the Blood Oxygen Sensor project.

Devices and Parts:

SNAP
PIC24 microcontroller
MAX30102 Sensor
RGB LCD display
Button switch
Links:

Public Functions:

// Function prototypes
void delay_ms(unsigned int ms);
void rgb_cmd(char Package);
void setReg(unsigned char reg, unsigned char dat);
void setRGB(unsigned char r, unsigned char g, unsigned char b);
void rgb_clr();
void rgb_home();
void noDisplay();
void display();
void blinkLED(void);
void noBlinkLED();
void setColorAll();
void setColorWhite();
void setColorRed();
void setCursor(uint8_t x, uint8_t y);
void grovergb_init(void);
void printStr(const char s[]);
void printChar(char myChar);

Initialization and Configuration:
grovergb_init(): Initializes the Grove RGB LCD module to be compatible with the PIC24 microcontroller using I2C communication.

Data Acquisition and Processing:
Not applicable in this document. (Related to the blood oxygen sensor functionality)

Buffer Management:
Not applicable in this document.

Control and Configuration:
setColorAll(): Turns off all colors on the display.
setColorWhite(): Sets the screen color to white.
setColorRed(): Sets the screen color to red.
blinkLED(): Flashes the specified LED registers.
noBlinkLED(): Stops the flashing of the LED registers.

Utility:
printStr(const char s[]): Prints a string to the display.
printChar(char myChar): Prints a character to the display.

Arguments and Outputs:
grovergb_init(): No arguments or outputs.
setColorAll(), setColorWhite(), setColorRed(), blinkLED(), noBlinkLED(): No arguments or outputs.
printStr(const char s[]): Takes a string s as input and prints it on the display.
printChar(char myChar): Takes a character myChar as input and prints it on the display.



Basic usage example- description - what device(s), part numbers, links, etc.
-basic readings

Advanced usage example - covering all the functions and features
How to build on what we have, switch modes, usage of green light,
Power saving mode, usage on different fingers,

References used:

Basic Background Video - https://www.youtube.com/watch?v=V5UvNVQsUsY

Sensor:


Guide to using Sensor - https://www.instructables.com/Guide-to-Using-MAX30102-Heart-Rate-and-Oxygen-Sens/

OxSensor Example Library
https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/src/MAX30105.h

Oxsensor Arduino Tutorial
https://lastminuteengineers.com/max30102-pulse-oximeter-heart-rate-sensor-arduino-tutorial/

Data Sheet
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf. 



LCD:
Grove LCD wiki
https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/ 

Grove LCD datasheet
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf

Other LCD datasheet 
https://files.seeedstudio.com/wiki/Grove-LCD_RGB_Backlight/Grove-LCD_RGB_Backlight_V5.0_Datasheet.pdf 

Other Other LCD datasheet
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/PCA9633.pdf 


](https://github.com/luketschepen/EE2361Final/tree/main 

INTRO- Description of the Library and its overall functions
Members: Dhayalan Balasubramanian, Luke Tschepen, Nicholas Kang, Cole Jaeger.

The Blood Oxygen Sensor project integrates the MAX30102 sensor for pulse oximetry and heart-rate monitoring, alongside the Grove RGB LCD module for versatile display feedback. Physical input is managed by the button_switch.c file, facilitating interaction with a button switch for toggling display and capturing button events. Crucially, the algorithm.c file computes heart rate and SpO2 levels using peak detection techniques and precise algorithms on sensor data. The I2C.h file handles essential I2C communication for interfacing with the sensor and other peripherals.

Separate Descriptions
MAX 30102 Description:
The code implements functionalities for interfacing with the MAX30102 sensor, which is a pulse oximeter and heart-rate sensor. It includes functions for initializing the sensor, configuring its settings, reading sensor data, processing the signals, and displaying the results.

The Grove RGB LCD module is a multifunctional display that integrates an RGB backlight and a character LCD. It is designed to provide a versatile display solution for various electronic projects. This document outlines the functionality and usage of the Grove RGB LCD module within the context of the Blood Oxygen Sensor project.

The button_switch.c file provides functionality for interfacing with a physical button switch in the Blood Oxygen Sensor project. It includes functions for initializing the button, detecting button presses, and determining the state of the button. The button switch is connected to RB7 (Pin 6) of the microcontroller, and its usage involves capturing button press events and toggling the display of heart rate information

The algorithm.c file is a crucial component of the Blood Oxygen Sensor project, dedicated to computing heart rate and SpO2 levels. It implements peak detection techniques on data from IR and red sensors to identify PPG signal peaks for heart rate determination. Additionally, it employs a precise algorithm to derive SpO2 values from the ratio of AC to DC components in the IR and red signals.

The I2C.h file contains functions for initializing and managing I2C communication on PIC24 microcontrollers. It includes functions for initializing I2C parameters, initiating start and stop conditions, writing and reading data over I2C, handling acknowledgments, and performing repeated start conditions.



Hardware description - what device(s), part numbers, links, etc.
SNAP: MP LAB SNAP PG164100
https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/UserGuides/50002787C.pdf 


PIC24: PIC24FJ64GA002
https://ww1.microchip.com/downloads/en/DeviceDoc/39881e.pdf 

Sensor: MAX30102
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf 

RGB LCD: Grove-LCD RGB Backlight JHD1313
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf 


Button: Square Tactile Pushbutton Switch SPST Momentary -ELSW-TPB-SPST
https://www.ckswitches.com/media/1462/pts125.pdf 

Full documentation- description - what device(s), part numbers, links, etc.

MAX 30102 Description:
The code implements functionalities for interfacing with the MAX30102 sensor, which is a pulse oximeter and heart-rate sensor. It includes functions for initializing the sensor, configuring its settings, reading sensor data, processing the signals, and displaying the results.

Devices and Parts:
SNAP
PIC24
MAX30102 Sensor
RGB LCD display
Button switch

Links to datasheets and part numbers:
SNAP: MP LAB SNAP PG164100
https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/UserGuides/50002787C.pdf 


PIC24: PIC24FJ64GA002
https://ww1.microchip.com/downloads/en/DeviceDoc/39881e.pdf 

Sensor: MAX30102
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf 

RGB LCD: Grove-LCD RGB Backlight JHD1313
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf 


Button: Square Tactile Pushbutton Switch SPST Momentary -ELSW-TPB-SPST
https://www.ckswitches.com/media/1462/pts125.pdf

Public Functions:
Initialization and Configuration:
max30102_init(void): Initializes the MAX30102 sensor.
max30102_setup_spo2(): Configures the MAX30102 sensor for SpO2 measurement.
max30102_setup_default(void): Sets up the MAX30102 sensor to the default state.
max30102_setup_display(void): Configures the display for showing SpO2 and heart rate data.

Data Acquisition and Processing:
max30102_process_signals(): Processes the sensor signals to calculate SpO2 and heart rate.
max30102_readTemp(): Reads the die temperature from the sensor.
calculateMovingAverage(uint32_t *buffer, int size): Calculates the moving average of the data buffer.

Buffer Management:
max30102_RED_putBuff(long int data): Inserts data into the Red LED buffer.
max30102_IR_putBuff(long int data): Inserts data into the Infrared LED buffer.
max30102_RED_getBuff(): Retrieves data from the Red LED buffer.
max30102_IR_getBuff(): Retrieves data from the Infrared LED buffer.

Control and Configuration:
Functions to configure various parameters such as LED mode, ADC range, sample rate, pulse width, FIFO settings, etc.


Utility:
max30102_shutdown(void): Shuts down the sensor.
softReset(void): Performs a soft reset of the sensor.

Arguments and Outputs:
Many functions take parameters to configure specific settings on the sensor, such as LED mode, sample rate, pulse width, etc.
The functions for inserting and retrieving data from the buffers (max30102_RED_putBuff, max30102_IR_putBuff, max30102_RED_getBuff, max30102_IR_getBuff) accept and return data values.
Functions for processing signals (max30102_process_signals()) calculate SpO2 and heart rate and store the results in global variables.
Display functions (max30102_setup_default, max30102_setup_display) output data to an RGB LCD display.




Grove RGB LCD Documentation

Description:

The Grove RGB LCD module is a multifunctional display that integrates an RGB backlight and a character LCD. It is designed to provide a versatile display solution for various electronic projects. This document outlines the functionality and usage of the Grove RGB LCD module within the context of the Blood Oxygen Sensor project.

Devices and Parts:

SNAP
PIC24 microcontroller
MAX30102 Sensor
RGB LCD display
Button switch
Links:

Public Functions:

// Function prototypes
void delay_ms(unsigned int ms);
void rgb_cmd(char Package);
void setReg(unsigned char reg, unsigned char dat);
void setRGB(unsigned char r, unsigned char g, unsigned char b);
void rgb_clr();
void rgb_home();
void noDisplay();
void display();
void blinkLED(void);
void noBlinkLED();
void setColorAll();
void setColorWhite();
void setColorRed();
void setCursor(uint8_t x, uint8_t y);
void grovergb_init(void);
void printStr(const char s[]);
void printChar(char myChar);

Initialization and Configuration:
grovergb_init(): Initializes the Grove RGB LCD module to be compatible with the PIC24 microcontroller using I2C communication.

Data Acquisition and Processing:
Not applicable in this document. (Related to the blood oxygen sensor functionality)

Buffer Management:
Not applicable in this document.

Control and Configuration:
setColorAll(): Turns off all colors on the display.
setColorWhite(): Sets the screen color to white.
setColorRed(): Sets the screen color to red.
blinkLED(): Flashes the specified LED registers.
noBlinkLED(): Stops the flashing of the LED registers.

Utility:
printStr(const char s[]): Prints a string to the display.
printChar(char myChar): Prints a character to the display.

Arguments and Outputs:
grovergb_init(): No arguments or outputs.
setColorAll(), setColorWhite(), setColorRed(), blinkLED(), noBlinkLED(): No arguments or outputs.
printStr(const char s[]): Takes a string s as input and prints it on the display.
printChar(char myChar): Takes a character myChar as input and prints it on the display.



Basic usage example- description - what device(s), part numbers, links, etc.
-basic readings

Advanced usage example - covering all the functions and features
How to build on what we have, switch modes, usage of green light,
Power saving mode, usage on different fingers,

References used:

Basic Background Video - https://www.youtube.com/watch?v=V5UvNVQsUsY

Sensor:


Guide to using Sensor - https://www.instructables.com/Guide-to-Using-MAX30102-Heart-Rate-and-Oxygen-Sens/

OxSensor Example Library
https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/src/MAX30105.h

Oxsensor Arduino Tutorial
https://lastminuteengineers.com/max30102-pulse-oximeter-heart-rate-sensor-arduino-tutorial/

Data Sheet
https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf. 



LCD:
Grove LCD wiki
https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/ 

Grove LCD datasheet
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/JHD1313%20FP-RGB-1%201.4.pdf

Other LCD datasheet 
https://files.seeedstudio.com/wiki/Grove-LCD_RGB_Backlight/Grove-LCD_RGB_Backlight_V5.0_Datasheet.pdf 

Other Other LCD datasheet
https://files.seeedstudio.com/wiki/Grove_LCD_RGB_Backlight/res/PCA9633.pdf 

PIC 24 Data Sheet:
https://ww1.microchip.com/downloads/en/DeviceDoc/39881e.pdf

PIC 24 I2C:
https://ww1.microchip.com/downloads/en/devicedoc/70000195f.pdf

)

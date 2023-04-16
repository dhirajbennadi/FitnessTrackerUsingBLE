/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include "em_gpio.h"

#define LED0_port  (gpioPortF)
#define LED0_pin   (4)
#define LED1_port  (gpioPortF)
#define LED1_pin   (5)

#define I2C_TEMP_SENSOR_ENABLE_PORT (gpioPortD)
#define I2C_TEMP_SENSOR_ENABLE_PIN  (15)

#define LCD_ENABLE_PORT (gpioPortD)
#define LCD_ENABLE_PIN  (15)

#define LCD_EXT_COMM_PORT (gpioPortD)
#define LCD_EXT_COMM_PIN  (13)

/*Push button PB0*/
#define PB0_port    (gpioPortF)
#define PB0_pin     (6)



// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void disableTempSensor();
void enableTempSensor();

void gpioSetDisplayExtcomin(bool value);
void gpioSensorEnSetOn();


#endif /* SRC_GPIO_H_ */

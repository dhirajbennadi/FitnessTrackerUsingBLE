/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#include "gpio.h"


// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:

	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);


  GPIO_DriveStrengthSet(I2C_TEMP_SENSOR_ENABLE_PORT, gpioDriveStrengthStrongAlternateStrong);
  //GPIO_DriveStrengthSet(I2C_TEMP_SENSOR_ENABLE_PORT, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(I2C_TEMP_SENSOR_ENABLE_PORT, I2C_TEMP_SENSOR_ENABLE_PIN,
                  gpioModePushPull, false);

  GPIO_DriveStrengthSet(LCD_EXT_COMM_PORT, gpioDriveStrengthStrongAlternateStrong);
  //GPIO_DriveStrengthSet(I2C_TEMP_SENSOR_ENABLE_PORT, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LCD_EXT_COMM_PORT, LCD_EXT_COMM_PIN,
                  gpioModePushPull, false);

  GPIO_PinModeSet(PB0_port, PB0_pin, gpioModeInputPullFilter, true);
  GPIO_ExtIntConfig (PB0_port, PB0_pin, PB0_pin, true, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

} // gpioInit()




void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}


void enableTempSensor()
{
  GPIO_PinOutSet(I2C_TEMP_SENSOR_ENABLE_PORT,
                 I2C_TEMP_SENSOR_ENABLE_PIN);
}


void disableTempSensor()
{
  GPIO_PinOutClear(I2C_TEMP_SENSOR_ENABLE_PORT,
                 I2C_TEMP_SENSOR_ENABLE_PIN);
}

void gpioSetDisplayExtcomin(bool value)
{
  if(value == true)
  {
      GPIO_PinOutSet(LCD_EXT_COMM_PORT, LCD_EXT_COMM_PIN);
  }
  else
  {
      GPIO_PinOutClear(LCD_EXT_COMM_PORT, LCD_EXT_COMM_PIN);
  }
}


void gpioSensorEnSetOn()
{
  GPIO_PinOutSet(LCD_ENABLE_PORT, LCD_ENABLE_PIN);
}

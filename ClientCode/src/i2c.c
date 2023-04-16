/*
 * i2c.c
 *
 *  Created on: April 28th, 2022
 *  Author: Dhiraj Bennadi & Karthik BR
 */


#include "i2c.h"
#include "sl_i2cspm.h"
#include "gpio.h"
#include "timers.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

/*Uncomment this Macro to print LOG_ERROR*/
//#define I2C_LOGGING_ERROR
//#define I2C_LOGGING_INFORMATION

#define TEMP_COMMAND_NO_HOLD_MASTER_MODE    (0xF3)
#define SI7021_SLAVE_ADDRESS                (0x40)
#define MAX_16_BIT_VAL                      (65536)

I2CSPM_Init_TypeDef stI2CInit;
I2C_TransferSeq_TypeDef stSeq;
I2C_TransferReturn_TypeDef i2cTransactionStatus;
uint8_t commandValueTempReading = TEMP_COMMAND_NO_HOLD_MASTER_MODE;
uint16_t recordRawTempValue = 0x00;

/*
 * This function initializes the I2C0 parameters
 *
 * Parameters: None
 *
 * Returns: None.
 */
void initializeParametersForI2C(void)
{
  stI2CInit.port = I2C0;
  stI2CInit.sclPort = gpioPortC;
  stI2CInit.sclPin = 10;
  stI2CInit.sdaPort = gpioPortC;
  stI2CInit.sdaPin = 11;
  stI2CInit.portLocationScl = 14;
  stI2CInit.portLocationSda = 16;
  stI2CInit.i2cRefFreq = 0;
  stI2CInit.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
  stI2CInit.i2cClhr = i2cClockHLRStandard;
}


#if 0
void getFrequency(void)
{
  uint32_t frequency;
  frequency = I2C_BusFreqGet(I2C0);

  for(int i = 0; i < 1000000; i++)
  {
    i++;
  }
}
#endif


/*
 * This function initializes the I2C0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void initI2C(void)
{
  initializeParametersForI2C();
  I2CSPM_Init(&stI2CInit);
}


/*
 * This function write the temp measurement command
 *
 * Parameters: None
 *
 * Returns: None.
 */
void writeTempCommand(void)
{
  I2C_TransferReturn_TypeDef i2cTransactionStatus;
  uint8_t commandValueTempReading = TEMP_COMMAND_NO_HOLD_MASTER_MODE;

  stSeq.addr = (SI7021_SLAVE_ADDRESS << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &commandValueTempReading;
  stSeq.buf[0].len = sizeof(commandValueTempReading);
  /*seq - address , flags , {data , length} */
  i2cTransactionStatus = I2CSPM_Transfer(I2C0, &stSeq);

  if(i2cTransactionStatus < i2cTransferDone)
  {
#ifdef I2C_LOGGING_ERROR
      LOG_ERROR("Failed to sending Command to Si7021\n\r");
      LOG_ERROR("Error Code = %d\n\r", i2cTransactionStatus);
#endif
  }
}


/*
 * This function reads the value of the temperature
 *
 * Parameters: None
 *
 * Returns: uint16_t
 */
uint16_t readTempCommand(void)
{
  uint16_t recordRawTempValue = 0x00;
  I2C_TransferReturn_TypeDef i2cTransactionStatus;

  stSeq.addr = (SI7021_SLAVE_ADDRESS << 1);
  stSeq.flags = I2C_FLAG_READ;
  stSeq.buf[0].data = (uint8_t*)&recordRawTempValue;
  stSeq.buf[0].len = sizeof(recordRawTempValue);
  i2cTransactionStatus = I2CSPM_Transfer(I2C0, &stSeq);

  if(i2cTransactionStatus < i2cTransferDone)
  {
#ifdef I2C_LOGGING_ERROR
      LOG_ERROR("Failed to sending Command to Si7021\n\r");
      LOG_ERROR("Error Code = %d\n\r", i2cTransactionStatus);
#endif
  }

  return recordRawTempValue;
}



/*
 * This function calculates the actual value of temperature from raw value
 *
 * Parameters: measuredTemp - Raw reading of temperature
 *
 * Returns: uint32_t
 */
uint32_t calculateActualTempFromRawValueIrq()
{
  uint32_t measuredTemp = 0;
  uint32_t actualTemp = 0U;

  measuredTemp |= (recordRawTempValue & 0xFF) << 8;
  measuredTemp |= (recordRawTempValue & 0xFF00) >> 8;

  /*Source: Datasheet Si7021*/
  /*Magic values used due to the reference to datasheet*/
  actualTemp = measuredTemp * 175.72;
  actualTemp = actualTemp / MAX_16_BIT_VAL;
  actualTemp -= 46.85;

  return actualTemp;

}

/*
 * This function calculates the actual value of temperature from raw value
 *
 * Parameters: measuredTemp - Raw reading of temperature
 *
 * Returns: uint32_t
 */
uint32_t calculateActualTempFromRawValue(uint16_t measuredTemp)
{
  uint32_t actualTemp = 0U;

  /*Source: Datasheet Si7021*/
  /*Magic values used due to the reference to datasheet*/
  actualTemp = measuredTemp * 175.72;
  actualTemp = actualTemp / MAX_16_BIT_VAL;
  actualTemp -= 46.85;

  return actualTemp;

}

/*Not Used for Assignment A4*/
#if 0
/*
 * This function performs the Temp measurement from the Si7021 device
 *
 * Parameters: None
 *
 * Returns: None
 */
void performTempMeasurement(void)
{
  uint16_t tempReadFromSensor = 0U;
  uint16_t measuredTemp = 0U;
  uint32_t actualTemp = 0U;

  enableTempSensor();
  timerWaitUs_polled(SI_7021_WAKE_UP_US);
  writeTempCommand();
  timerWaitUs_polled(SI_7021_TEMP_CONVERSION_US);
  tempReadFromSensor = readTempCommand();

  disableTempSensor();

  /*Extracting MSB and LSV from raw Value*/
  measuredTemp |= (tempReadFromSensor & 0xFF) << 8;
  measuredTemp |= (tempReadFromSensor & 0xFF00) >> 8;

  actualTemp = calculateActualTempFromRawValue(measuredTemp);


#ifdef I2C_LOGGING_INFORMATION
  LOG_INFO("Actual Temperature = %d\n\r", actualTemp);
#endif

}
#endif

/*This functions turns on the temp sensor and starts the timer with
 * COMP1 Interrupt
 */
void turnPowerOnAndSetupTimer(void)
{
  enableTempSensor();
  timerWaitUs_irq(SI_7021_WAKE_UP_US);
}


/*
 * This function sends the temperature command to Si 7021
 *
 * Parameters: None
 *
 * Returns: None
 */
void writeTempCommandIrq(void)
{

  stSeq.addr = (SI7021_SLAVE_ADDRESS << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &commandValueTempReading;
  stSeq.buf[0].len = sizeof(commandValueTempReading);

  NVIC_ClearPendingIRQ(I2C0_IRQn);
  NVIC_EnableIRQ(I2C0_IRQn);

  /*seq - address , flags , {data , length} */
  i2cTransactionStatus = I2C_TransferInit(I2C0, &stSeq);

  if(i2cTransactionStatus < i2cTransferDone)
  {
#ifdef I2C_LOGGING_ERROR
      LOG_ERROR("Failed to sending Command to Si7021\n\r");
      LOG_ERROR("Error Code = %d\n\r", i2cTransactionStatus);
#endif
  }

}

/*
 * This function reads the value of the temperature
 *
 * Parameters: None
 *
 * Returns: uint16_t
 */
void readTempCommandIrq(void)
{

  stSeq.addr = (SI7021_SLAVE_ADDRESS << 1);
  stSeq.flags = I2C_FLAG_READ;
  stSeq.buf[0].data = (uint8_t*)&recordRawTempValue;
  stSeq.buf[0].len = sizeof(recordRawTempValue);

  NVIC_ClearPendingIRQ(I2C0_IRQn);
  NVIC_EnableIRQ(I2C0_IRQn);

  /*seq - address , flags , {data , length} */
  i2cTransactionStatus = I2C_TransferInit(I2C0, &stSeq);

  if(i2cTransactionStatus < i2cTransferDone)
  {
#ifdef I2C_LOGGING_ERROR
      LOG_ERROR("Failed to sending Command to Si7021\n\r");
      LOG_ERROR("Error Code = %d\n\r", i2cTransactionStatus);
#endif
  }

}


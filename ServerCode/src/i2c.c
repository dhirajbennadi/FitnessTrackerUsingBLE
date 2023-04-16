/*
 * i2c.c
 *
 *  Created on: April 28th 2022
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

#define SGP40                               (0x59)

I2CSPM_Init_TypeDef stI2CInit;
I2C_TransferSeq_TypeDef stSeq;
I2C_TransferReturn_TypeDef i2cTransactionStatus;
uint8_t commandValueTempReading = TEMP_COMMAND_NO_HOLD_MASTER_MODE;


/*0x32 Accelerometer Zout L*/
/*0x06 Accelerometer Power Management register*/

uint8_t IMUEmptyCommand = 0x7F;

uint8_t PowerOnAccerlermeterRegsiter = 0x06;
uint8_t PowerOnAccelerometerData = 0x01;

uint16_t recordRawTempValue = 0x00;

//uint8_t IMUReadValue = 0x00;

uint8_t powerManagementValue = 0;
uint8_t registerBankValue = 0;

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


#if 0
/*
 * This function sends the temperature command to Si 7021
 *
 * Parameters: None
 *
 * Returns: None
 */
void writeIMUCommand(void)
{

  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &IMUEmptyCommand;
  stSeq.buf[0].len = sizeof(IMUEmptyCommand);

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
void readIMUCommand(void)
{

  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_READ;
  stSeq.buf[0].data = (uint8_t*)&IMUReadValue;
  stSeq.buf[0].len = sizeof(IMUReadValue);

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

#endif




uint8_t PowerOnAccelerometerArray1[2] = {0x06 , 0x80};
//uint8_t PowerOnAccelerometerArray2 = 0x02;

void powerONAccelerometer(void)
{
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &PowerOnAccelerometerArray1[0];
  stSeq.buf[0].len = sizeof(uint16_t);
  //stSeq.buf[1].data = &PowerOnAccelerometerArray2; // 0x01
  //stSeq.buf[1].len = sizeof(PowerOnAccelerometerArray2);

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

uint8_t PowerOnAccelerometerArray2[2] = {0x06 , 0x02};


void sendPowerOnData(void)
{
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &PowerOnAccelerometerArray2[0];
  stSeq.buf[0].len = sizeof(uint16_t);

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

/**************Z Register Values***********/

uint8_t Accel_ZRegLowWrite = 0x32;
uint8_t Accel_ZRegLowRead = 0;

uint8_t Accel_ZRegHighWrite = 0x31;
uint8_t Accel_ZRegHighRead = 0;

bool firstReadZ = false;

void readRegisterZ(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadZ == false)
    {
      firstReadZ = true;
      stSeq.buf[0].data = &Accel_ZRegLowWrite;
      stSeq.buf[0].len = sizeof(Accel_ZRegLowWrite);
      Accel_ZRegLowRead = 0;
      stSeq.buf[1].data = &Accel_ZRegLowRead;
      stSeq.buf[1].len = sizeof(Accel_ZRegLowRead);
    }
  else
    {
      firstReadZ = false;
      stSeq.buf[0].data = &Accel_ZRegHighWrite;
      stSeq.buf[0].len = sizeof(Accel_ZRegHighWrite);
      Accel_ZRegHighRead = 0;
      stSeq.buf[1].data = &Accel_ZRegHighRead;
      stSeq.buf[1].len = sizeof(Accel_ZRegHighRead);

    }

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


void regsiterZValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Accel_ZRegLowRead;
  *highRegVal = Accel_ZRegHighRead;

}


/**************Y Register Values***********/

uint8_t Accel_YRegLowWrite = 0x30;
uint8_t Accel_YRegLowRead = 0;

uint8_t Accel_YRegHighWrite = 0x2F;
uint8_t Accel_YRegHighRead = 0;

bool firstReadY = false;

void readRegisterY(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadY == false)
    {
      firstReadY = true;
      stSeq.buf[0].data = &Accel_YRegLowWrite;
      stSeq.buf[0].len = sizeof(Accel_YRegLowWrite);
      Accel_YRegLowRead = 0;
      stSeq.buf[1].data = &Accel_YRegLowRead;
      stSeq.buf[1].len = sizeof(Accel_YRegLowRead);
    }
  else
    {
      firstReadY = false;
      stSeq.buf[0].data = &Accel_YRegHighWrite;
      stSeq.buf[0].len = sizeof(Accel_YRegHighWrite);
      Accel_YRegHighRead = 0;
      stSeq.buf[1].data = &Accel_YRegHighRead;
      stSeq.buf[1].len = sizeof(Accel_YRegHighRead);

    }

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

void regsiterYValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Accel_YRegLowRead;
  *highRegVal = Accel_YRegHighRead;

}

/**************X Register Values***********/

uint8_t Accel_XRegLowWrite = 0x2E;
uint8_t Accel_XRegLowRead = 0;

uint8_t Accel_XRegHighWrite = 0x2D;
uint8_t Accel_XRegHighRead = 0;

bool firstReadX = false;

void readRegisterX(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadX == false)
    {
      firstReadX = true;
      stSeq.buf[0].data = &Accel_XRegLowWrite;
      stSeq.buf[0].len = sizeof(Accel_XRegLowWrite);
      Accel_XRegLowRead = 0;
      stSeq.buf[1].data = &Accel_XRegLowRead;
      stSeq.buf[1].len = sizeof(Accel_XRegLowRead);
    }
  else
    {
      firstReadX = false;
      stSeq.buf[0].data = &Accel_XRegHighWrite;
      stSeq.buf[0].len = sizeof(Accel_XRegHighWrite);
      Accel_XRegHighRead = 0;
      stSeq.buf[1].data = &Accel_XRegHighRead;
      stSeq.buf[1].len = sizeof(Accel_XRegHighRead);

    }

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

void regsiterXValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Accel_XRegLowRead;
  *highRegVal = Accel_XRegHighRead;

}

/**********************Register Bank**************/

uint8_t RegisterBank2Data[2] = {0x7F , 0x20};
uint8_t RegisterBank0Data[2] = {0x7F , 0x00};



void sendRegisterBankData(uint8_t registerBank)
{
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  if(registerBank == 2)
    {
      stSeq.buf[0].data = &RegisterBank2Data[0];
      stSeq.buf[0].len = sizeof(uint16_t);
    }
  else if(registerBank == 0)
    {
      stSeq.buf[0].data = &RegisterBank0Data[0];
      stSeq.buf[0].len = sizeof(uint16_t);
    }


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
 * This function sends the temperature command to Si 7021
 *
 * Parameters: None
 *
 * Returns: None
 */

uint8_t pwrMgmt = 0x06;
uint8_t regBank = 0x7F;
void writeIMUCommand(uint8_t registerValue)
{

  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;

  if(registerValue == 0x06)
    {
      stSeq.buf[0].data = &pwrMgmt;
      stSeq.buf[0].len = sizeof(pwrMgmt);
    }
  else if(registerValue == 0x7F)
    {
      stSeq.buf[0].data = &regBank;
      stSeq.buf[0].len = sizeof(regBank);
    }

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
void readIMUCommand(uint8_t *registerReadValue)
{

  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_READ;
  stSeq.buf[0].data = (uint8_t*)registerReadValue;
  stSeq.buf[0].len = sizeof(uint8_t);

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

uint8_t readRegisterBankValue(void)
{

  uint8_t actualValue = 0;

  actualValue = registerBankValue;

  return actualValue;

}

uint8_t readPowerManagementValue(void)
{

  uint8_t actualValue = 0;

  actualValue = powerManagementValue;

  return actualValue;

}


/****************Configure Accerlerometer****************/
uint8_t configureAccerlerometerData[2] = {0x14 , 0x01};


void sendAccelConfigData(void)
{
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE;
  stSeq.buf[0].data = &configureAccerlerometerData[0];
  stSeq.buf[0].len = sizeof(uint16_t);

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



/*******************GyroScope Read Registers*****************/
/*******************GyroScope Read Registers*****************/
/*******************GyroScope Read Registers*****************/
/*******************GyroScope Read Registers*****************/
uint8_t Gyro_ZRegLowWrite = 0x38;
uint8_t Gyro_ZRegLowRead = 0;

uint8_t Gyro_ZRegHighWrite = 0x37;
uint8_t Gyro_ZRegHighRead = 0;

bool firstReadGyro_Z = false;

void GyroReadRegisterZ(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadGyro_Z == false)
    {
      firstReadZ = true;
      stSeq.buf[0].data = &Gyro_ZRegLowWrite;
      stSeq.buf[0].len = sizeof(Gyro_ZRegLowWrite);
      Gyro_ZRegLowRead = 0;
      stSeq.buf[1].data = &Gyro_ZRegLowRead;
      stSeq.buf[1].len = sizeof(Gyro_ZRegLowRead);
    }
  else
    {
      firstReadGyro_Z = false;
      stSeq.buf[0].data = &Gyro_ZRegHighWrite;
      stSeq.buf[0].len = sizeof(Gyro_ZRegHighWrite);
      Gyro_ZRegHighRead = 0;
      stSeq.buf[1].data = &Gyro_ZRegHighRead;
      stSeq.buf[1].len = sizeof(Gyro_ZRegHighRead);

    }

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


void GyroRegsiterZValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Gyro_ZRegLowRead;
  *highRegVal = Gyro_ZRegHighRead;

}


/**************Y Register Values***********/

uint8_t Gyro_YRegLowWrite = 0x36;
uint8_t Gyro_YRegLowRead = 0;

uint8_t Gyro_YRegHighWrite = 0x35;
uint8_t Gyro_YRegHighRead = 0;

bool firstReadGyro_Y = false;

void GyroReadRegisterY(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadGyro_Y == false)
    {
      firstReadGyro_Y = true;
      stSeq.buf[0].data = &Gyro_YRegLowWrite;
      stSeq.buf[0].len = sizeof(Gyro_YRegLowWrite);
      Gyro_YRegLowRead = 0;
      stSeq.buf[1].data = &Gyro_YRegLowRead;
      stSeq.buf[1].len = sizeof(Gyro_YRegLowRead);
    }
  else
    {
      firstReadGyro_Y = false;
      stSeq.buf[0].data = &Gyro_YRegHighWrite;
      stSeq.buf[0].len = sizeof(Gyro_YRegHighWrite);
      Gyro_YRegHighRead = 0;
      stSeq.buf[1].data = &Gyro_YRegHighRead;
      stSeq.buf[1].len = sizeof(Gyro_YRegHighRead);

    }

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


void GyroRegsiterYValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Gyro_YRegLowRead;
  *highRegVal = Gyro_YRegHighRead;

}

/**************X Register Values***********/

uint8_t Gyro_XRegLowWrite = 0x34;
uint8_t Gyro_XRegLowRead = 0;

uint8_t Gyro_XRegHighWrite = 0x33;
uint8_t Gyro_XRegHighRead = 0;

bool firstReadGyro_X = false;

void GyroReadRegisterX(void)
{
  /*Indicate combined write/read sequence: S+ADDR(W)+DATA0+Sr+ADDR(R)+DATA1+P.*/
  stSeq.addr = (0x68 << 1);
  stSeq.flags = I2C_FLAG_WRITE_READ;
  if(firstReadGyro_X == false)
    {
      firstReadGyro_X = true;
      stSeq.buf[0].data = &Gyro_XRegLowWrite;
      stSeq.buf[0].len = sizeof(Gyro_XRegLowWrite);
      Gyro_XRegLowRead = 0;
      stSeq.buf[1].data = &Gyro_XRegLowRead;
      stSeq.buf[1].len = sizeof(Gyro_XRegLowRead);
    }
  else
    {
      firstReadGyro_X = false;
      stSeq.buf[0].data = &Gyro_XRegHighWrite;
      stSeq.buf[0].len = sizeof(Gyro_XRegHighWrite);
      Gyro_XRegHighRead = 0;
      stSeq.buf[1].data = &Gyro_XRegHighRead;
      stSeq.buf[1].len = sizeof(Gyro_XRegHighRead);

    }

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


void GyroRegsiterXValue(uint8_t *lowRegVal, uint8_t *highRegVal)
{
  *lowRegVal = Gyro_XRegLowRead;
  *highRegVal = Gyro_XRegHighRead;

}



/********************************************************************/
uint8_t sgp40Command[8] = {0x26,0x0F,0x80,0x00,0xA2,0x66,0x66,0x93};

void writeTempCommandIrqSGP40(void)
{

  stSeq.addr = (SGP40 << 1);
  stSeq.flags = I2C_FLAG_WRITE_WRITE;
  stSeq.buf[0].data = sgp40Command;
  stSeq.buf[0].len = sizeof(sgp40Command);

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


uint16_t sgp40ReadValue = 0;

void readTempCommandIrqSGP40(void)
{

  stSeq.addr = (SGP40 << 1);
  stSeq.flags = I2C_FLAG_READ;
  stSeq.buf[0].data = (uint8_t*)&sgp40ReadValue;
  stSeq.buf[0].len = sizeof(sgp40ReadValue);

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


uint16_t calculateAirQualityFromRawSGP40(void)
{
  uint32_t airQualityValue = 0;

  airQualityValue |= (sgp40ReadValue & 0xFF) << 8;
  airQualityValue |= (sgp40ReadValue & 0xFF00) >> 8;

  return airQualityValue;

}

/*
 * i2c.h
 *
 *  Created on: April 28th 2022
 *  Author: Dhiraj Bennadi & Karthik BR
 */



#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdint.h>
#include "i2c.h"
#include "sl_i2cspm.h"
#include "gpio.h"
#include "timers.h"

/*Reference: Data Sheet of Si 7021*/
#define SI_7021_WAKE_UP_US                  (80000)
#define SI_7021_TEMP_CONVERSION_US          (10800)

void initI2C(void);
void getFrequency(void);
void performTempMeasurement(void);

void turnPowerOnAndSetupTimer(void);
void writeTempCommandIrq(void);
void readTempCommandIrq(void);
uint32_t calculateActualTempFromRawValue(uint16_t measuredTemp);
uint32_t calculateActualTempFromRawValueIrq(void);

extern I2CSPM_Init_TypeDef stI2CInit;
extern I2C_TransferSeq_TypeDef stSeq;
extern I2C_TransferReturn_TypeDef i2cTransactionStatus;
extern uint8_t commandValueTempReading;

extern uint16_t recordRawTempValue;

/*IMU Commands*/
void writeIMUCommand(uint8_t registerValue);
void readIMUCommand(uint8_t *registerReadValue);
extern uint8_t IMUEmptyCommand;
extern uint8_t IMUReadValue;
uint32_t whoAmIValue(void);
void powerONAccelerometer(void);
extern uint8_t PowerOnAccerlermeterRegsiter;
extern uint8_t PowerOnAccelerometerData;
void sendPowerOnData(void);

/*Register X Value*/
void readRegisterX(void);
void regsiterXValue(uint8_t *lowRegVal, uint8_t *highRegVal);

/*Register Y*/
void readRegisterY(void);
void regsiterYValue(uint8_t *lowRegVal, uint8_t *highRegVal);

/*Register Z*/
void readRegisterZ(void);
void regsiterZValue(uint8_t *lowRegVal, uint8_t *highRegVal);

/*Register Bank Data*/
void sendRegisterBankData(uint8_t registerBank);


/*MPU Register*/
uint8_t MPU_WhoAMIValue(void);
void readWHOAMIRegister(void);
void writeWHOAMIRegister(void);
extern uint8_t WHOAMIRegsiterAddress;
extern uint8_t WHOAMIRegsiterData;

extern uint8_t powerManagementValue;
extern uint8_t registerBankValue;
uint8_t readPowerManagementValue(void);
uint8_t readRegisterBankValue(void);

/*Configure Accelerometer Value */
void sendAccelConfigData(void);


/*Gyroscope*/
void GyroReadRegisterZ(void);
void GyroReadRegisterY(void);
void GyroReadRegisterX(void);

void GyroRegsiterZValue(uint8_t *lowRegVal, uint8_t *highRegVal);
void GyroRegsiterYValue(uint8_t *lowRegVal, uint8_t *highRegVal);
void GyroRegsiterXValue(uint8_t *lowRegVal, uint8_t *highRegVal);




/*****SGP40**************/
void writeTempCommandIrqSGP40(void);
void readTempCommandIrqSGP40(void);
uint16_t calculateAirQualityFromRawSGP40(void);
#endif /* SRC_I2C_H_ */

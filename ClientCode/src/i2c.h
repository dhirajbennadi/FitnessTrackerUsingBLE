/*
 * i2c.h
 *
 *  Created on: April 28th, 2022
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
#endif /* SRC_I2C_H_ */

/*
 * scheduler.h
 *
 *  Created on: April 28th 2022
 *  Author: Dhiraj Bennadi & Karthik BR
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdint.h>
#include "sl_bt_api.h"

typedef enum{
  NO_EVENT = 0,
  evtLETIMER0_UF = 1,
  evtLETIMER0_COMP1 = 2,
  evtI2C_Complete = 4,
  evtI2C_NACK = 8,
  evtGPIO_IRQ = 16,
  numofEvents
}interruptEvents;

typedef enum{
  Idle = 0,
  PowerOn7021 = 1,
  I2CTempWriteCommand = 2,
  MeasurementInProgress = 3,
  I2CTempReadCommand = 4,
  PowerOnAccelerometer = 5,
  SendPowerOnData = 6,
  TempDelay = 7,
  writeRegisterBank = 8,
  readRegisterBank = 9,
  SendRegisterBankData = 10,
  ChangeRegisterBankValue = 11,
  configureAccerlerometerValues = 12,
  numStates
}stateMachineStates;

typedef enum{
  IdleRead = 0,
  ReadXregisterLow = 1,
  ReadYregisterLow = 2,
  ReadZregisterLow = 3,
  ReadXregisterHigh = 4,
  ReadYregisterHigh = 5,
  ReadZregisterHigh = 6,
  WaitState = 7,
  ReadXGyroRegisterLow = 8,
  ReadYGyroRegisterLow = 9,
  ReadZGyroRegisterLow = 10,
  ReadXGyroRegisterHigh = 11,
  ReadYGyroRegisterHigh = 12,
  ReadZGyroRegisterHigh = 13,
  WaitStateGyro = 14,

  AirQualityWriteI2C = 15,
  AirQualityWriteWait = 16,
  AirQualityReadI2C = 17,
  AirQualityReadWait = 18,
  AirQualityWait = 19,

  TotalReads
}IMUReadingStates;


/*Reference : Client Table Assignment 6*/
typedef enum{
  DiscoveryService = 0,
  GATT_ServiceComplete = 1,
  GATT_CharacteristicComplete = 2,
  GATT_CharacteristicConfirmation = 3,
  DiscoveryStateMachinenumStates
}discoveryStateMachine;

typedef enum {
  noEvent = 0,
  client_connectionOpenedevt = 1,
  client_gattServiceCompletedevt = 2,
  client_characteristicValueIndication = 3,
  client_connectionClosedevt = 4
}clientEvents;
/*
 * This function returns the events set by the scheduler as per the priority
 *
 * Parameters: None
 *
 * Returns: uint32_t - Event to be processed
 */
uint32_t getNextEvent(void);

void schedulerSetEvent_LETIMER(void);
void schedulerSetEvent_LETIMERCompare1(void);
void schedulerSetEvent_I2CComplete(void);
void schedulerSetEvent_I2CNACK(void);
void schedulerSetEvent_GPIO(void);


void TempStateMachine(sl_bt_msg_t *evt);
void DiscoveryStateMachine(sl_bt_msg_t *evt);


extern int eventVariable;

//void IMUStateMachine(void);
void IMUStateMachine(sl_bt_msg_t *evt);
void IMUStateMachineForReadingValues(sl_bt_msg_t *evt);

extern uint8_t shiftStateMachine;

extern int eventVariable;


void MPU6050_StateMachine(void);
#endif /* SRC_SCHEDULER_H_ */

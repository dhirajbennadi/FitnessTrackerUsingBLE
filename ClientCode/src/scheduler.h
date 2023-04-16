/*
 * scheduler.h
 *
 *  Created on: April 28th, 2022
 *  Author: Dhiraj Bennadi
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
  evtGPIO1_IRQ = 32,
  numofEvents
}interruptEvents;

typedef enum{
  Idle = 0,
  PowerOn7021 = 1,
  I2CTempWriteCommand = 2,
  MeasurementInProgress = 3,
  I2CTempReadCommand = 4,
  numStates
}stateMachineStates;


/*Reference : Client Table Assignment 6*/
typedef enum{
  DiscoveryService = 0,
  GATT_ServiceComplete = 1,
  GATT_CharacteristicComplete = 2,
  GATT_CharacteristicConfirmation = 3,
  DiscoveryServiceButton = 4,
  GATT_ButtonServiceComplete = 5,
  GATT_ButtonCharacteristicComplete = 6,

  /*Accelerometer Services*/
  DiscoveryServiceAccelerometer = 7,
  GATT_AccelerometerServiceComplete = 8,
  GATT_AcclerometerCharacteristicComplete = 9,

  /*Gyroscope Services*/
  DiscoveryServiceGyroscope = 10,
  GATT_GyroscopeServiceComplete = 11,
  GATT_GyroscopeCharacteristicComplete = 12,

  /*Air Quality Services*/
  DiscoveryServiceAirQuality = 13,
  GATT_AirQualityServiceComplete = 14,
  GATT_AirQualityCharacteristicComplete = 15,

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
void schedulerSetEvent_GPIO1(void);


void TempStateMachine(sl_bt_msg_t *evt);
void DiscoveryStateMachine(sl_bt_msg_t *evt);

extern int eventVariable;
#endif /* SRC_SCHEDULER_H_ */

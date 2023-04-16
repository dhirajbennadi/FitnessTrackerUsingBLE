/*
 * scheduler.c
 *
 *  Created on: April 28th 2022
 *  Author: Dhiraj Bennadi & Karthik BR
 */

#include <stdbool.h>
#include "scheduler.h"
#include "app.h"
#include "em_core.h"
#include "i2c.h"
#include "sl_power_manager.h"
#include "ble.h"
#include "sl_bt_api.h"
#include "math.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

uint8_t shiftStateMachine = 0;

/*
 * This function returns the events set by the scheduler as per the priority
 *
 * Parameters: None
 *
 * Returns: uint32_t - Event to be processed
 */
// scheduler routine to return 1 event to main()code and clear that event
uint32_t getNextEvent(void)
{
 uint32_t theEvent = 0;
 uint32_t copyofEventVariable = 0;


 CORE_DECLARE_IRQ_STATE;
 // enter critical section
 CORE_ENTER_CRITICAL();
 // clear the event in your data structure, this has to be a read-modify-write
 copyofEventVariable = eventVariable;
 eventVariable = 0;
 // exit critical section
 CORE_EXIT_CRITICAL();

 /*Returning the event based on the priority*/
 /*The logic checks the number of bits needed to identify an event*/
// while(copyofEventVariable)
// {
//     bitSet = copyofEventVariable & 0x01;
//     bitCounter++;
//     if(bitSet)
//     {
//         break;
//     }
//     copyofEventVariable = copyofEventVariable >> 1;
// }


#ifdef LOGGING_ENABLE
 LOG_ERROR("Event Variable = %d\n\r", eventVariable);
#endif
 switch(copyofEventVariable)
 {
   case evtLETIMER0_UF:
     theEvent = evtLETIMER0_UF;
#ifdef LOGGING_ENABLE
     LOG_INFO("Underflow Interrupt Returned from Scheduler\n\r");
#endif
     break;

   case evtLETIMER0_COMP1:
     theEvent = evtLETIMER0_COMP1;
#ifdef LOGGING_ENABLE
     LOG_INFO("COMP1 Interrupt Returned from Scheduler\n\r");
#endif
     break;

   case evtI2C_Complete:
     theEvent = evtI2C_Complete;
#ifdef LOGGING_ENABLE
     LOG_INFO("I2C Interrupt Returned from Scheduler\n\r");
#endif
     break;

   case evtI2C_NACK:
     theEvent = evtI2C_NACK;
     break;

   default:
     theEvent = NO_EVENT;
     break;
 }

   return (theEvent);
} // getNextEvent()


/*
 * This function set a scheduler event - LETIMER
 *
 * Parameters: None
 *
 * Returns: None.
 */
// scheduler routine to set a scheduler event
void schedulerSetEvent_LETIMER(void)
{

  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtLETIMER0_UF;
  sl_bt_external_signal(evtLETIMER0_UF);
  // exit critical section
  CORE_EXIT_CRITICAL();
} // schedulerSetEvent_LETIMER()

/*
 * This function set a scheduler event - LETIMER
 *
 * Parameters: None
 *
 * Returns: None.
 */
void schedulerSetEvent_LETIMERCompare1(void)
{

  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtLETIMER0_COMP1;
  sl_bt_external_signal(evtLETIMER0_COMP1);
  // exit critical section
  CORE_EXIT_CRITICAL();
} // schedulerSetEvent_LETIMERCompare1()


/*
 * This function set a scheduler event - I2C Complete
 *
 * Parameters: None
 *
 * Returns: None.
 */
void schedulerSetEvent_I2CComplete(void)
{
  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtI2C_Complete;
  sl_bt_external_signal(evtI2C_Complete);
  // exit critical section
  CORE_EXIT_CRITICAL();
} //schedulerSetEvent_I2CComplete

void schedulerSetEvent_I2CNACK(void)
{
  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtI2C_Complete;
  sl_bt_external_signal(evtI2C_NACK);
  // exit critical section
  CORE_EXIT_CRITICAL();
} //schedulerSetEvent_I2CNACK

void schedulerSetEvent_GPIO(void)
{
  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtI2C_Complete;
  sl_bt_external_signal(evtGPIO_IRQ);
  // exit critical section
  CORE_EXIT_CRITICAL();
}

/*
 * This function handles the state machine for reading a temperature value from
 * Si 7021 through LETIMER and I2C Interrupts
 *
 * Parameters: None
 *
 * Returns: None.
 */
void TempStateMachine(sl_bt_msg_t *evt)
{
  static stateMachineStates currentState = Idle;
  uint32_t actualTempReading = 0;
  uint32_t event = 0;
  uint32_t currentEventType = 0;

  ble_data_struct_t *ble_DataPtr = getBleDataPtr();

  if(ble_DataPtr->connectionStatus == false)
  {
    return;
  }

  currentEventType = SL_BT_MSG_ID(evt->header);

  if(currentEventType == sl_bt_evt_system_external_signal_id)
  {
      event = evt->data.evt_system_external_signal.extsignals;
  }
  else
  {
    return;
  }

  switch(currentState)
  {
    case Idle:
      currentState = Idle;
      if(event == evtLETIMER0_UF)
      {
          enableTempSensor();
          timerWaitUs_irq(SI_7021_WAKE_UP_US);
          currentState = PowerOn7021;
      }
      break;

    case PowerOn7021:
      currentState = PowerOn7021;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          writeTempCommandIrq();
          currentState = I2CTempWriteCommand;
      }
      break;

    case I2CTempWriteCommand:
      currentState = I2CTempWriteCommand;
      if(event == evtI2C_Complete)
      {
          NVIC_DisableIRQ(I2C0_IRQn);
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          timerWaitUs_irq(SI_7021_TEMP_CONVERSION_US);
          currentState = MeasurementInProgress;
      }
      else if(event == evtI2C_NACK)
      {
          /*If NACK repeat, resend the command*/
          writeTempCommandIrq();
          currentState = I2CTempWriteCommand;
      }
      break;

    case MeasurementInProgress:
      currentState = MeasurementInProgress;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          readTempCommandIrq();
          currentState = I2CTempReadCommand;
      }
      break;

    case I2CTempReadCommand:
      currentState = I2CTempReadCommand;
      if(event == evtI2C_Complete)
      {
          //disableTempSensor();
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          actualTempReading = calculateActualTempFromRawValueIrq();
          updateTempValueInBleDatabase(actualTempReading);
          //updateTempValueInBleDatabase(20);
          NVIC_DisableIRQ(I2C0_IRQn);
#ifdef LOGGING_ENABLE
          LOG_INFO("Timestamp = %d Temp Reading = %d\n\r",loggerGetTimestamp() , actualTempReading);
#endif
          currentState = Idle;
      }
      else if(event == evtI2C_NACK)
      {
          readTempCommandIrq();
          currentState = I2CTempReadCommand;
      }
      break;

    case numStates:
      /*Remove the warning*/
      break;

    default:
      /*Default Case*/
      break;

  }
}

void DiscoveryStateMachine(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  static discoveryStateMachine currentState = DiscoveryService;

  ble_data_struct_t *ble_DataPtr = getBleDataPtr();

  //uint32_t currentEventType = SL_BT_MSG_ID(evt->header);

  switch(currentState)
  {
    case DiscoveryService:
      currentState = DiscoveryService;

      if(ble_DataPtr->bleClientEvent == client_connectionOpenedevt)
        {
          sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_DataPtr->connectionStatus,
                                                            sizeof(thermo_service),
                                                            (const uint8_t * ) thermo_service);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack Discover primary services error\n\r");
            }
          currentState = GATT_ServiceComplete;

        }
      else if(ble_DataPtr->bleClientEvent == client_connectionClosedevt)
        {
          currentState = DiscoveryService;
        }

      break;

    case GATT_ServiceComplete:
      currentState = GATT_ServiceComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_DataPtr->connectionStatus,
                                                           ble_DataPtr->tempServiceHandle,
                                                           sizeof(thermo_char),
                                                           (const uint8_t * ) thermo_char);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Discover Characteristics Error\n\r");
            }

          currentState = GATT_CharacteristicComplete;

        }
      else if(ble_DataPtr->bleClientEvent == client_connectionClosedevt)
        {
          currentState = DiscoveryService;
        }
      break;

    case GATT_CharacteristicComplete:
      currentState = GATT_CharacteristicComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                          ble_DataPtr->tempCharacteristicHandle,
                                                          sl_bt_gatt_indication);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Set Characteristic Notification Error\n\r");
            }

          currentState = GATT_CharacteristicConfirmation;
        }
      else if(ble_DataPtr->bleClientEvent == client_connectionClosedevt)
        {
          currentState = DiscoveryService;
        }
      break;

    case GATT_CharacteristicConfirmation:
      currentState = GATT_CharacteristicConfirmation;
      if(ble_DataPtr->bleClientEvent == client_characteristicValueIndication)
        {
          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Send Characteristic Confirmation Error\n\r");
            }

          currentState = GATT_CharacteristicConfirmation;
        }
      else if(ble_DataPtr->bleClientEvent == client_connectionClosedevt)
        {
          currentState = DiscoveryService;
        }
      break;

    default:
      break;
  }
  ble_DataPtr->bleClientEvent = noEvent;
}


void IMUStateMachine(sl_bt_msg_t *evt)
{
  static stateMachineStates currentState = Idle;
  uint8_t powerManagement = 0;
  uint8_t registerBank = 0;
  uint32_t currentEventType = 0;

  uint8_t powerManagementRegister = 0x06;
  uint8_t registerBankRegister = 0x7F;
  uint32_t event = 0;
  //event = getNextEvent();

  ble_data_struct_t *ble_DataPtr = getBleDataPtr();

  if(ble_DataPtr->connectionStatus == false)
  {
    return;
  }

  /*TODO:Dhiraj Bonded Status*/
  if(ble_DataPtr->bondedStatus != true)
  {
      return;
  }

  currentEventType = SL_BT_MSG_ID(evt->header);

  if(currentEventType == sl_bt_evt_system_external_signal_id)
  {
      event = evt->data.evt_system_external_signal.extsignals;
  }
  else
  {
    return;
  }

  switch(currentState)
  {
    case Idle:
      currentState = Idle;
      if(event == evtLETIMER0_UF)
      {
          //enableTempSensor();
          //timerWaitUs_irq(SI_7021_WAKE_UP_US);
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          powerONAccelerometer();
          currentState = TempDelay;
          //currentState = PowerOn7021;

          //currentState = SendPowerOnData;
      }
      break;

    case TempDelay:
      currentState = TempDelay;
      if(event == evtI2C_Complete)
      {
          timerWaitUs_irq(120000);
          //sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          currentState = PowerOnAccelerometer;
      }
      break;

    case PowerOnAccelerometer:
      currentState = PowerOnAccelerometer;
      if(event == evtLETIMER0_COMP1)
        {
          sendPowerOnData();
          currentState = SendPowerOnData;

        }
      break;

    case SendPowerOnData:
      currentState = SendPowerOnData;
      if(event == evtI2C_Complete)
        {
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          timerWaitUs_irq(SI_7021_WAKE_UP_US);
          currentState = PowerOn7021;
        }
      break;

    case PowerOn7021:
      currentState = PowerOn7021;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          writeIMUCommand(powerManagementRegister);
          currentState = I2CTempWriteCommand;
      }
      break;

    case I2CTempWriteCommand:
      currentState = I2CTempWriteCommand;
      if(event == evtI2C_Complete)
      {
          NVIC_DisableIRQ(I2C0_IRQn);
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          timerWaitUs_irq(SI_7021_TEMP_CONVERSION_US);
          currentState = MeasurementInProgress;
      }
      break;

    case MeasurementInProgress:
      currentState = MeasurementInProgress;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          readIMUCommand(&powerManagementValue);
          //currentState = I2CTempReadCommand;
          currentState = SendRegisterBankData;
      }
      break;

    case SendRegisterBankData:
      currentState = SendRegisterBankData;
      if(event == evtI2C_Complete)
        {
          /*Changed the register bank to 2*/
          sendRegisterBankData(0x02);
          currentState = configureAccerlerometerValues;

        }
      break;

    case configureAccerlerometerValues:
      currentState = configureAccerlerometerValues;
      if(event == evtI2C_Complete)
        {
          sendAccelConfigData();
          currentState = ChangeRegisterBankValue;
        }
      break;

    case ChangeRegisterBankValue:
      currentState = ChangeRegisterBankValue;
      if(event == evtI2C_Complete)
        {
          sendRegisterBankData(0x00);
          currentState = writeRegisterBank;
        }
      break;


    case writeRegisterBank:
      currentState = writeRegisterBank;
      if(event == evtI2C_Complete)
        {
          /*Write Register Bank*/
          writeIMUCommand(registerBankRegister);
          currentState = readRegisterBank;
        }
      break;

    case readRegisterBank:
      currentState = readRegisterBank;
      if(event == evtI2C_Complete)
        {
          /*Write Register Bank*/
          readIMUCommand(&registerBankValue);
          currentState = I2CTempReadCommand;
        }
      break;

    case I2CTempReadCommand:
      currentState = I2CTempReadCommand;
      if(event == evtI2C_Complete)
      {
          //disableTempSensor();
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          //actualTempReading = calculateActualTempFromRawValueIrq();
          powerManagement = readPowerManagementValue();
          registerBank = readRegisterBankValue();
          NVIC_DisableIRQ(I2C0_IRQn);
          LOG_INFO("Timestamp = %d PWR_Mgmt = 0x%x RegisterBank = 0x%x\n\r",
                   loggerGetTimestamp() , powerManagement, registerBank);

          if(powerManagement == 0x02 && registerBank == 0x00)
            {
              shiftStateMachine = 1;
              LOG_INFO("First State Machine Complete\n\r");
            }

          //shiftStateMachine = 1;

          currentState = Idle;
      }
      break;

    case numStates:
      /*Remove the warning*/
      break;

    default:
      /*Default Case*/
      break;

  }
}


void IMUStateMachineForReadingValues(sl_bt_msg_t *evt)
{
  static IMUReadingStates currentState = IdleRead;
  uint32_t event = 0;
  uint8_t registerXValueLow = 0;
  uint8_t registerYValueLow = 0;
  uint8_t registerZValueLow = 0;
  uint8_t registerXValueHigh = 0;
  uint8_t registerYValueHigh = 0;
  uint8_t registerZValueHigh = 0;
  int16_t AcceleroXValue = 0;
  int16_t AcceleroYValue = 0;
  int16_t AcceleroZValue = 0;


  float AcceleroXValueDisplay = 0;
  float AcceleroYValueDisplay = 0;
  float AcceleroZValueDisplay = 0;

  uint8_t GyroregisterXValueLow = 0;
  uint8_t GyroregisterYValueLow = 0;
  uint8_t GyroregisterZValueLow = 0;
  uint8_t GyroregisterXValueHigh = 0;
  uint8_t GyroregisterYValueHigh = 0;
  //uint8_t GyroregisterZValueHigh = 0;
  int16_t GyroXValue = 0;
  //int16_t GyroYValue = 0;
  //int16_t GyroZValue = 0;

  //uint8_t airQualityLowValue = 0;
  //uint8_t airQualityHighValue = 0;
  uint16_t airQualityIndex = 0;
  //event = getNextEvent();
  uint32_t currentEventType = 0;
  ble_data_struct_t *ble_DataPtr = getBleDataPtr();

  if(ble_DataPtr->connectionStatus == false)
  {
    return;
  }

  /*TODO:Dhiraj Bonded Status*/
  if(ble_DataPtr->bondedStatus != true)
  {
      return;
  }

  currentEventType = SL_BT_MSG_ID(evt->header);

  if(currentEventType == sl_bt_evt_system_external_signal_id)
  {
      event = evt->data.evt_system_external_signal.extsignals;
  }
  else
  {
    return;
  }

  switch(currentState)
  {
    case IdleRead:
      currentState = IdleRead;

      if(event == evtLETIMER0_UF)
        {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          readRegisterX();
          currentState = ReadXregisterLow;
        }
      break;

    case ReadXregisterLow:
      currentState = ReadXregisterLow;
      if(event == evtI2C_Complete)
        {
          readRegisterX();
          currentState = ReadXregisterHigh;
        }
      break;


    case ReadXregisterHigh:
      currentState = ReadXregisterHigh;
      if(event == evtI2C_Complete)
        {
          readRegisterY();
          currentState = ReadYregisterLow;
        }
      break;

    case ReadYregisterLow:
      currentState = ReadYregisterLow;
      if(event == evtI2C_Complete)
        {
          readRegisterY();
          currentState = ReadYregisterHigh;
        }
      break;


    case ReadYregisterHigh:
      currentState = ReadYregisterHigh;
      if(event == evtI2C_Complete)
        {
          readRegisterZ();
          currentState = ReadZregisterLow;
        }
      break;


    case ReadZregisterLow:
      currentState = ReadZregisterLow;
      if(event == evtI2C_Complete)
        {
          readRegisterZ();
          currentState = ReadZregisterHigh;
        }
      break;

    case ReadZregisterHigh:
      currentState = ReadZregisterHigh;
      if(event == evtI2C_Complete)
        {
          //readRegisterZ();
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          NVIC_DisableIRQ(I2C0_IRQn);
          currentState = WaitState;
          timerWaitUs_irq(20000);
        }
      break;

    case WaitState:
      currentState = WaitState;
      if(event == evtLETIMER0_COMP1)
        {

          regsiterXValue(&registerXValueLow, &registerXValueHigh);
          regsiterYValue(&registerYValueLow, &registerYValueHigh);
          regsiterZValue(&registerZValueLow, &registerZValueHigh);

          AcceleroXValue = (registerXValueHigh << 8) | (registerXValueLow);
          AcceleroYValue = (registerYValueHigh << 8) | (registerYValueLow);
          AcceleroZValue = (registerZValueHigh << 8) | (registerZValueLow);


          AcceleroXValue += 1200;
          AcceleroYValue += 1200;
          //AcceleroZValue -= 800;


//          AcceleroXValue = ~AcceleroXValue + 1;
//          AcceleroYValue = ~AcceleroYValue + 1;
//          AcceleroZValue = ~AcceleroZValue + 1;

//          LOG_INFO("Raw Accelerometer X = %dg, Y = %dg , Z = %dg \n\r",
//                   AcceleroXValue,
//                   AcceleroYValue,
//                   AcceleroZValue);

//          LOG_INFO("Timestamp = %d Accelerometer X = %ld g, Accelerometer Y = %ld g , Accelerometer Z = %ld g \n\r",
//                    loggerGetTimestamp() ,
//                    AcceleroXValue,
//                    AcceleroYValue,
//                    AcceleroZValue);


          AcceleroXValueDisplay = (AcceleroXValue / 16384.0);
          AcceleroYValueDisplay = (AcceleroYValue / 16384.0);
          AcceleroZValueDisplay = (AcceleroZValue / 16384.0);



//
//          LOG_INFO("Accelerometer X = %f g, Y Y = %f g , Z = %f g \n\r",
//                    AcceleroXValueDisplay,
//                    AcceleroYValueDisplay,
//                    AcceleroZValueDisplay);

//          float pitchAngle = atan((AcceleroYValueDisplay * AcceleroYValueDisplay  + 0.0)  / (sqrt((AcceleroXValueDisplay*AcceleroXValueDisplay)
//                                                                + (AcceleroZValueDisplay*AcceleroZValueDisplay) + 0.0)));


          float pitchAngle = 0.0;

          pitchAngle = atan2(AcceleroXValueDisplay , sqrt((AcceleroYValueDisplay*AcceleroYValueDisplay) +
                                                          (AcceleroZValueDisplay*AcceleroZValueDisplay))) * 180 ;

          pitchAngle = pitchAngle + 0.0 / 3.14;

          //LOG_INFO("Pitch Angle = %f\n\r", pitchAngle);

          float resultant = 0.0;
          resultant = sqrt((AcceleroXValueDisplay*AcceleroXValueDisplay) +
                           (AcceleroYValueDisplay*AcceleroYValueDisplay) +
                           (AcceleroZValueDisplay*AcceleroZValueDisplay));

          float tilt = 0.0;

          tilt = acos(AcceleroZValueDisplay + 0.0 / resultant) * 180;
          tilt = tilt / 3.14;

          //LOG_INFO("Tilt Angle = %f\n\r", tilt);

          AcceleroZValue = AcceleroZValue / 4096;


          LOG_INFO("Accelerometer Tilt = %f\n\r", tilt + 0.0);

          //updateTempValueInBleDatabase(20);
          updateAccelerometerValueInBleDatabase(tilt);

//                    LOG_INFO("Timestamp = %d Accelerometer Z = %ld g \n\r",
//                              loggerGetTimestamp() , AcceleroZValue);





//          float roll, pitch, resultant;
//          float tilt;
//
//          roll = atan2(AcceleroYValueDisplay, AcceleroZValueDisplay)*180/3.14;
//          //LOG("\n\r roll: %f", roll);
//          pitch = atan2(AcceleroXValueDisplay, sqrt(AcceleroYValueDisplay*AcceleroYValueDisplay + AcceleroZValueDisplay*AcceleroZValueDisplay))*180/3.14;
//          //LOG("\n\r pitch: %f", pitch);
//
//          resultant = sqrt((AcceleroXValueDisplay * AcceleroXValueDisplay) + (AcceleroYValueDisplay * AcceleroYValueDisplay) + (AcceleroZValueDisplay * AcceleroZValueDisplay));
//
//          tilt = acos(AcceleroYValueDisplay / resultant) * 180/3.14;

//          if(AcceleroXValue > 2)
//            {
//              AcceleroXValue -= 2;
//            }
//
//          if(AcceleroYValue > 2)
//            {
//              AcceleroYValue -= 2;
//            }
//
//          if(AcceleroZValue > 2)
//            {
//              AcceleroZValue -= 2;
//            }

//                    LOG_INFO("Timestamp = %d Tilt = %f \n\r",
//                             loggerGetTimestamp() , tilt);

//          LOG_INFO("Timestamp = %d Accelerometer X = %f g, Accelerometer Y = %f g , Accelerometer Z = %f g \n\r",
//                   loggerGetTimestamp() ,
//                   AcceleroXValue,
//                   AcceleroYValue,
//                   AcceleroZValue);

          //AcceleroXValue = AcceleroXValue / 16384;
          //AcceleroYValue = AcceleroYValue / 16384;
          //AcceleroZValue = AcceleroZValue / 16384;

//          LOG_INFO("Timestamp = %d Accelerometer X = %d, Accelerometer Y = %d , Accelerometer Z = %d \n\r",
//                   loggerGetTimestamp() ,
//                   AcceleroXValue,
//                   AcceleroYValue,
//                   AcceleroZValue);
          //LOG_INFO("Timestamp = %d Accelerometer Y = %d\n\r",loggerGetTimestamp() , AcceleroYValue);
          //LOG_INFO("Timestamp = %d Accelerometer Z = %d\n\r",loggerGetTimestamp() , AcceleroZValue);


          currentState = ReadXGyroRegisterLow;
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          GyroReadRegisterX();
        }
      break;

      /************************Gyroscope Values*****************/
    case ReadXGyroRegisterLow:
      currentState = ReadXGyroRegisterLow;
      if(event == evtI2C_Complete)
        {
          GyroReadRegisterX();
          currentState = ReadXGyroRegisterHigh;
        }
      break;


    case ReadXGyroRegisterHigh:
      currentState = ReadXGyroRegisterHigh;
      if(event == evtI2C_Complete)
        {
          GyroReadRegisterY();
          currentState = ReadYGyroRegisterLow;
        }
      break;

    case ReadYGyroRegisterLow:
      currentState = ReadYGyroRegisterLow;
      if(event == evtI2C_Complete)
        {
          GyroReadRegisterY();
          currentState = ReadYGyroRegisterHigh;
        }
      break;


    case ReadYGyroRegisterHigh:
      currentState = ReadYGyroRegisterHigh;
      if(event == evtI2C_Complete)
        {
          GyroReadRegisterZ();
          currentState = ReadZGyroRegisterLow;
        }
      break;


    case ReadZGyroRegisterLow:
      currentState = ReadZGyroRegisterLow;
      if(event == evtI2C_Complete)
        {
          GyroReadRegisterZ();
          currentState = ReadZGyroRegisterHigh;
        }
      break;


    case ReadZGyroRegisterHigh:
      currentState = ReadZGyroRegisterHigh;
      if(event == evtI2C_Complete)
        {
          //readRegisterZ();
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          NVIC_DisableIRQ(I2C0_IRQn);
          currentState = WaitStateGyro;
          timerWaitUs_irq(10000);
        }
      break;

    case WaitStateGyro:
      currentState = WaitStateGyro;
      if(event == evtLETIMER0_COMP1)
        {
          GyroRegsiterXValue(&GyroregisterXValueLow, &GyroregisterXValueHigh);
          GyroRegsiterYValue(&GyroregisterYValueLow, &GyroregisterYValueHigh);
          GyroRegsiterZValue(&GyroregisterZValueLow, &GyroregisterYValueHigh);

          GyroXValue = (GyroregisterYValueHigh << 8) | (GyroregisterXValueLow);
          //GyroYValue = (GyroregisterYValueHigh << 8) | (GyroregisterYValueLow);
          //GyroZValue = (GyroregisterYValueHigh << 8) | (GyroregisterZValueLow);



//          LOG_INFO("Raw Gyro X = %dg,  Y = %dg ,  Z = %dg \n\r",
//                   GyroXValue,
//                   GyroYValue,
//                   GyroZValue);

          float gyroXDisplayValue = 0.0;
          //float gyroYDisplayValue = 0.0;
          //float gyroZDisplayValue = 0.0;

          gyroXDisplayValue = GyroXValue / 131.0;
          //gyroYDisplayValue = GyroYValue / 131.0;
          //gyroZDisplayValue = GyroZValue / 131.0;

//
//          LOG_INFO("Calibrated Gyro X = %f deg,  Y = %f deg ,  Z = %f deg \n\r",
//                   gyroXDisplayValue,
//                   gyroYDisplayValue,
//                   gyroZDisplayValue);

//          float xAngle = 0.0;
//          float yAngle = 0.0;
//          float zAngle = 0.0;
//
//          xAngle = (180/3.14) * (atan2(gyroYDisplayValue , gyroZDisplayValue) + 3.14);
//          yAngle = (180/3.14) * (atan2(gyroXDisplayValue , gyroZDisplayValue) + 3.14);
//          zAngle = (180/3.14) * (atan2(gyroXDisplayValue , gyroYDisplayValue) + 3.14);

          updateGyroscopeValueInBleDatabase(gyroXDisplayValue);


          /*TODO:Danger Change for Air Quality Index*****************************/
          //currentState = IdleRead;

          writeTempCommandIrqSGP40();
          currentState = AirQualityWriteI2C;
        }
      break;


    case AirQualityWriteI2C:
      currentState = AirQualityWriteI2C;
      if(event == evtI2C_Complete)
        {
          timerWaitUs_irq(50000);
          currentState = AirQualityWriteWait;
        }
      break;


    case AirQualityWriteWait:
      currentState = AirQualityWriteWait;
      if(event == evtLETIMER0_COMP1)
        {
          readTempCommandIrqSGP40();
          currentState = AirQualityReadI2C;
        }
      break;

    case AirQualityReadI2C:
      currentState = AirQualityReadI2C;
      if(event == evtI2C_Complete)
        {
          timerWaitUs_irq(50000);
          currentState = AirQualityWait;
        }
      break;

    case AirQualityWait:
      currentState = AirQualityWait;
      if(event == evtLETIMER0_COMP1)
        {


          airQualityIndex = calculateAirQualityFromRawSGP40();

          //LOG_INFO("Air Quality Index Raw= %d\n\r", airQualityIndex);
          airQualityIndex = airQualityIndex / 1000;
          //LOG_INFO("Air Quality Index Processed= %d\n\r", airQualityIndex);

          updateAirQualityIndexValueInBleDatabase(airQualityIndex);
          currentState = IdleRead;
        }
      break;

#if 0

    case ReadXregisterLow:
      currentState = ReadXregisterLow;
      if(event == evtI2C_Complete)
        {
          //sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          //registerXValue = regsiterXValue();
          NVIC_DisableIRQ(I2C0_IRQn);
          LOG_INFO("Timestamp = %d X Temp low register value = %X\n\r",loggerGetTimestamp() , registerXValue);
          currentState = ReadXDelay;
          timerWaitUs_irq(20000);

        }
      break;

    case ReadXDelay:
      currentState = ReadXDelay;

      if(event == evtLETIMER0_COMP1)
        {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          readRegisterX();
          currentState = ReadYregister;

        }
      break;

    case ReadYregister:
      currentState = ReadYregister;
      if(event == evtI2C_Complete)
        {
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          registerYValue = regsiterYValue();
          NVIC_DisableIRQ(I2C0_IRQn);
          LOG_INFO("Timestamp = %d Temp Y low register value = %X\n\r",loggerGetTimestamp() , registerYValue);
          //currentState = IdleRead;
          currentState = ReadYDelay;
          timerWaitUs_irq(20000);
        }
      break;

    case ReadYDelay:
      currentState = ReadYDelay;

      if(event == evtLETIMER0_COMP1)
        {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          readRegisterZ();
          currentState = ReadZregister;
        }
      break;

    case ReadZregister:
      currentState = ReadZregister;

      if(event == evtI2C_Complete)
        {
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          registerZValue = regsiterZValue();
          NVIC_DisableIRQ(I2C0_IRQn);
          //LOG_INFO("Timestamp = %d Z low register value = %d\n\r",loggerGetTimestamp() , registerZValue);
          currentState = IdleRead;
        }
      break;
#endif

    default:
      break;
  }
}


void MPU6050_StateMachine(void)
{
  static stateMachineStates currentState = Idle;
  uint8_t WHOAMIValue = 0;
  uint32_t event = 0;
  event = getNextEvent();

  switch(currentState)
  {
    case Idle:
      currentState = Idle;
      if(event == evtLETIMER0_UF)
      {
          //enableTempSensor();
          timerWaitUs_irq(SI_7021_WAKE_UP_US);
          currentState = PowerOn7021;
      }
      break;

    case PowerOn7021:
      currentState = PowerOn7021;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          //writeTempCommandIrq();
          writeWHOAMIRegister();
          currentState = I2CTempWriteCommand;
      }
      break;

    case I2CTempWriteCommand:
      currentState = I2CTempWriteCommand;
      if(event == evtI2C_Complete)
      {
          NVIC_DisableIRQ(I2C0_IRQn);
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          timerWaitUs_irq(SI_7021_TEMP_CONVERSION_US);
          currentState = MeasurementInProgress;
      }
      break;

    case MeasurementInProgress:
      currentState = MeasurementInProgress;
      if(event == evtLETIMER0_COMP1)
      {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          //readTempCommandIrq();
          readWHOAMIRegister();
          currentState = I2CTempReadCommand;
      }
      break;

    case I2CTempReadCommand:
      currentState = I2CTempReadCommand;
      if(event == evtI2C_Complete)
      {
          //disableTempSensor();
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          WHOAMIValue = MPU_WhoAMIValue();
          NVIC_DisableIRQ(I2C0_IRQn);
          LOG_INFO("Timestamp = %d Temp Reading = %d\n\r",loggerGetTimestamp() , WHOAMIValue);
          currentState = Idle;
      }
      break;

    case numStates:
      /*Remove the warning*/
      break;

    default:
      /*Default Case*/
      break;

  }

}

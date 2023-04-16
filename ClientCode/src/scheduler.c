/*
 * scheduler.c
 *
 *  Created on: April 28th, 2022
 *  Author: Dhiraj Bennadi
 */
#include <stdbool.h>
#include "scheduler.h"
#include "app.h"
#include "em_core.h"
#include "i2c.h"
#include "sl_power_manager.h"
#include "ble.h"
#include "sl_bt_api.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

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

void schedulerSetEvent_GPIO1(void)
{
  CORE_DECLARE_IRQ_STATE;
  // enter critical section
  CORE_ENTER_CRITICAL();
  // set the event in your data structure, this has to be a read-modify-write
  //eventVariable |= evtI2C_Complete;
  sl_bt_external_signal(evtGPIO1_IRQ);
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
  //static discoveryStateMachine currentState = DiscoveryService;

  static discoveryStateMachine currentState = DiscoveryServiceAccelerometer;

  ble_data_struct_t *ble_DataPtr = getBleDataPtr();

  (void)evt;

  if(ble_DataPtr->bleClientEvent == client_connectionClosedevt)
    {
      //currentState = DiscoveryService;
      currentState = DiscoveryServiceAccelerometer;
    }

  if(ble_DataPtr->bondedStatus != true)
    {
      return;
    }


//  if(ble_DataPtr->button1Status == sl_bt_gatt_disable)
//    {
//      currentState = GATT_ButtonCharacteristicComplete;
//    }


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

          //currentState = GATT_CharacteristicConfirmation;

          /*TODO: Dhiraj Major Change due to button service not being activated in Final project*/
          //currentState = DiscoveryServiceButton;
          currentState = DiscoveryServiceAccelerometer;
        }
      break;

    case GATT_CharacteristicConfirmation:
      currentState = GATT_CharacteristicConfirmation;
      if(ble_DataPtr->bleClientEvent == client_characteristicValueIndication)
        {

          LOG_INFO("Discovery State Machine E3\n\r");
          //currentState = DiscoveryServiceGyroscope;

//          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
//          if(sc != SL_STATUS_OK)
//            {
//              LOG_INFO("BL:Stack GATT Send Characteristic Confirmation Error\n\r");
//            }
//
          currentState = GATT_CharacteristicConfirmation;

#ifdef DANGER_CHANGES
          /*TODO:Dhiraj Bennadi Danger Change*/
          if(ble_DataPtr->button1Status == 0)
            {

              ble_DataPtr->notificationsDisabled = false;


              sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                              ble_DataPtr->accelerometerCharactersticHandle,
                                                              sl_bt_gatt_disable);
              if (!(sc == SL_STATUS_OK))
                {
                  LOG_ERROR("Error in button_state PB1 set characteristic 0 = %X\r\n", sc);
                }

              ble_DataPtr->button1Status = sl_bt_gatt_indication;

              //currentState =

            }
          else
            {

              sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
              if(sc != SL_STATUS_OK)
                {
                  LOG_INFO("BL:Stack GATT Send Characteristic Confirmation Error\n\r");
                }

              currentState = GATT_CharacteristicConfirmation;
            }
#endif

        }
      break;


    case DiscoveryServiceButton:
      currentState = DiscoveryServiceButton;

      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_DataPtr->connectionStatus,
                                                            sizeof(button_service),
                                                            (const uint8_t * ) button_service);


          currentState = GATT_ButtonServiceComplete;
        }
      break;

    case GATT_ButtonServiceComplete:
      currentState = GATT_ButtonServiceComplete;

      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_DataPtr->connectionStatus,
                                                           ble_DataPtr->buttonServiceHandle,
                                                           sizeof(button_char),
                                                           (const uint8_t * ) button_char);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Discover Characteristics Error\n\r");
            }

          currentState = GATT_ButtonCharacteristicComplete;

        }
      break;

    case GATT_ButtonCharacteristicComplete:
      currentState = GATT_ButtonCharacteristicComplete;

      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                          ble_DataPtr->buttonCharacteristicHandle,
                                                          sl_bt_gatt_indication);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Set Characteristic Notification Error\n\r");
            }

          //currentState = GATT_CharacteristicConfirmation;
          currentState = GATT_CharacteristicConfirmation;

        }
      break;


    case DiscoveryServiceAccelerometer:
      currentState = DiscoveryServiceAccelerometer;
      if(ble_DataPtr->bleClientEvent == client_connectionOpenedevt)
        {

          LOG_INFO("Discovery State Machine E7\n\r");
          sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_DataPtr->connectionStatus,
                                                            sizeof(accelerometer_service),
                                                            (const uint8_t * ) accelerometer_service);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack Discover primary services error\n\r");
            }

          currentState = GATT_AccelerometerServiceComplete;
        }
      break;

    case GATT_AccelerometerServiceComplete:
      currentState = GATT_AccelerometerServiceComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          LOG_INFO("Discovery State Machine E8\n\r");
          sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_DataPtr->connectionStatus,
                                                           ble_DataPtr->AccelerometerServiceHandle,
                                                           sizeof(accelerometer_char),
                                                           (const uint8_t * ) accelerometer_char);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Discover Characteristics Error\n\r");
            }
          currentState = GATT_AcclerometerCharacteristicComplete;
        }
      break;

    case GATT_AcclerometerCharacteristicComplete:
      currentState = GATT_AcclerometerCharacteristicComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {

          LOG_INFO("Discovery State Machine E9\n\r");
          /*TODO:Dhiraj Bennadi, Danger Change*/

          sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                          ble_DataPtr->accelerometerCharactersticHandle,
                                                          sl_bt_gatt_indication);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Set Characteristic Notification Error\n\r");
            }


          /*TODO: Major Change Danger Danger*/
          //currentState = GATT_CharacteristicConfirmation;

          currentState = DiscoveryServiceGyroscope;
        }
      break;



      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/
      /*************************Gyroscope****************************************/
      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/
    case DiscoveryServiceGyroscope:
      currentState = DiscoveryServiceGyroscope;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          LOG_INFO("Discovery State Machine E10\n\r");
          sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_DataPtr->connectionStatus,
                                                            sizeof(gyroscope_service),
                                                            (const uint8_t * ) gyroscope_service);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack Discover primary services error\n\r");
            }

          currentState = GATT_GyroscopeServiceComplete;
        }
      break;

    case GATT_GyroscopeServiceComplete:
      currentState = GATT_GyroscopeServiceComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          LOG_INFO("Discovery State Machine E11\n\r");
          sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_DataPtr->connectionStatus,
                                                           ble_DataPtr->GyroscopeServiceHandle,
                                                           sizeof(gyroscope_char),
                                                           (const uint8_t * ) gyroscope_char);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Discover Characteristics Error\n\r");
            }
          currentState = GATT_GyroscopeCharacteristicComplete;
        }
      break;

    case GATT_GyroscopeCharacteristicComplete:
      currentState = GATT_GyroscopeCharacteristicComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {

          LOG_INFO("Discovery State Machine E12\n\r");
          /*TODO:Dhiraj Bennadi, Danger Change*/

          sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                          ble_DataPtr->GyroscopeCharactersticHandle,
                                                          sl_bt_gatt_indication);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Set Characteristic Notification Error\n\r");
            }

          /*Danger Change*/
          //currentState = GATT_CharacteristicConfirmation;

          currentState = DiscoveryServiceAirQuality;

        }
      break;


      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/
      /*************************Air Qualtiy****************************************/
      /**************************************************************************/
      /**************************************************************************/
      /**************************************************************************/

    case DiscoveryServiceAirQuality:
      currentState = DiscoveryServiceAirQuality;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          LOG_INFO("Discovery State Machine E13\n\r");
          sc = sl_bt_gatt_discover_primary_services_by_uuid(ble_DataPtr->connectionStatus,
                                                            sizeof(airQualityIndex_service),
                                                            (const uint8_t * ) airQualityIndex_service);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack Discover primary services error\n\r");
            }

          currentState = GATT_AirQualityServiceComplete;
        }
      break;

    case GATT_AirQualityServiceComplete:
      currentState = GATT_AirQualityServiceComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {
          LOG_INFO("Discovery State Machine E14\n\r");
          sc = sl_bt_gatt_discover_characteristics_by_uuid(ble_DataPtr->connectionStatus,
                                                           ble_DataPtr->AirQualityServiceHandle,
                                                           sizeof(airQualityIndex_char),
                                                           (const uint8_t * ) airQualityIndex_char);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Discover Characteristics Error\n\r");
            }
          currentState = GATT_AirQualityCharacteristicComplete;
        }
      break;

    case GATT_AirQualityCharacteristicComplete:
      currentState = GATT_AirQualityCharacteristicComplete;
      if(ble_DataPtr->bleClientEvent == client_gattServiceCompletedevt)
        {

          LOG_INFO("Discovery State Machine E15\n\r");
          /*TODO:Dhiraj Bennadi, Danger Change*/

          sc = sl_bt_gatt_set_characteristic_notification(ble_DataPtr->connectionStatus,
                                                          ble_DataPtr->AirQualityCharactersticHandle,
                                                          sl_bt_gatt_indication);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack GATT Set Characteristic Notification Error\n\r");
            }

          currentState = GATT_CharacteristicConfirmation;
        }
      break;



    default:
      break;
  }
  ble_DataPtr->bleClientEvent = noEvent;
}

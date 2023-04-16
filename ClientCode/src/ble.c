/*
 * ble.c
 *
 *  Created on: April 28th, 2022
 *  Author: Dhiraj Bennadi
 */

#include "ble.h"
#include "lcd.h"
#include "ble_device_type.h"
#include "scheduler.h"
#include "sl_bgapi.h"
#include "math.h"
#include "gpio.h"
#include "circular_buffer.h"



#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define ADVERTISING_MIN_INTERVAL    (0x190)
#define ADVERTISING_MAX_INTERVAL    (0x190)
#define ADVERTISING_DEFAULT_VALUE   (0x00)

#define CONNECTION_MIN_INTERVAL     (0x3C)
#define CONNECTION_MAX_INTERVAL     (0x3C)
#define CONNECTION_LATENCY          (0x04)
#define CONNECTION_TIMEOUT          (75)
#define CONNECTION_MIN_CE_LENGTH    (0x00)
#define CONNECTION_MAX_CE_LENGTH    (0xFFFF)

#define NO_OFFSET                   (0x0)
#define ACCEPT_BONDING_REQUEST      (0x1)
#define REJECT_BONDING_REQUEST      (0x0)

#define ACCEPT_CONNECTION_REQUEST   (0x1)
#define REJECT_CONNECTION_REQUEST   (0x0)

#define BUTTON_PRESSED              (0x00)
#define BUTTON_RELEASED             (0x01)

#define LOGGING_BLE_STACK 1

#define ADVERTISING_SET_HANDLE      (0xFF)
#if DEVICE_IS_BLE_SERVER
#else
static int32_t FLOAT_TO_INT32(const uint8_t *value_start_little_endian);
#endif
// Health Thermometer service UUID defined by Bluetooth SIG
const uint8_t thermo_service[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
const uint8_t thermo_char[2] = { 0x1c, 0x2a };

// Button Press service UUID
const uint8_t button_service[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00};
// Button Press characteristic UUID
const uint8_t button_char[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00};

// fed4e930-d7a0-4a7a-aab1-965408f1a410
const uint8_t accelerometer_service[16] = {0x10, 0xa4, 0xf1, 0x08, 0x54, 0x96, 0xb1, 0xaa, 0x7a, 0x4a, 0xa0, 0xd7, 0x30,0xe9, 0xd4, 0xfe};
//9fb90b37-d582-46a1-82d5-adc5717d9795
const uint8_t accelerometer_char[16] = {0x95 , 0x97, 0x7d, 0x71, 0xc5, 0xad, 0xd5, 0x82, 0xa1, 0x46, 0x82, 0xd5, 0x37, 0x0b, 0xb9, 0x9f};

//9d468196-7c30-4e6c-9e5f-6df95a91843c
const uint8_t gyroscope_service[16] = {0x3c, 0x84, 0x91, 0x5a, 0xf9, 0x6d, 0x5f, 0x9e, 0x6c, 0x4e, 0x30, 0x7c, 0x96,0x81, 0x46, 0x9d};
//ab95f4e2-684d-4459-8a98-ae3769d9c4a4
const uint8_t gyroscope_char[16] = {0xa4 , 0xc4, 0xd9, 0x69, 0x37, 0xae, 0x98, 0x8a, 0x59, 0x44, 0x4d, 0x68, 0xe2, 0xf4, 0x95, 0xab};

//52a48cdb-14c0-4544-9daa-7a985f9d1731
const uint8_t airQualityIndex_service[16] = {0x31, 0x17, 0x9d, 0x5f, 0x98, 0x7a, 0xaa, 0x9d, 0x44, 0x45, 0xc0, 0x14, 0xdb, 0x8c, 0xa4, 0x52};

//ad0e70d8-d206-4566-b7ea-03b5a85a0ab7
const uint8_t airQualityIndex_char[16] = {0xb7, 0x0a, 0x5a, 0xa8, 0xb5, 0x03, 0xea, 0xb7, 0x66, 0x45, 0x06, 0xd2, 0xd8, 0x70, 0x0e, 0xad};

#if DEVICE_IS_BLE_SERVER
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertisingSetHandle = ADVERTISING_SET_HANDLE;
#endif

ble_data_struct_t ble_data;

//static circularBuffer_t buttonIndicationNode;
static circularBuffer_Stats buttonIndicationNodesStats;


ble_data_struct_t* getBleDataPtr()
{
  return (&ble_data);
} // getBleDataPtr()

/***************************************************************************//**
  void handle_ble_event(sl_bt_msg_t *evt);
 * Description: void handle_ble_event is called from sl_bt_on_event for every cycle.
 *              This works on the states of events of bluetooth.
 ******************************************************************************/

void handle_ble_event(sl_bt_msg_t *evt)
{

  sl_status_t sc;
  uint32_t passKey;

#if DEVICE_IS_BLE_SERVER
  uint8_t system_id[8];
#else
  int tempValueInt;
  uint8_t *tempValueptr;

  /*Accelerometer Service Value*/
  int16_t acclerometerValue;
  uint8_t *accelerometerValueptr;

  /*Accelerometer Service Value*/
  int16_t gyroscopeValue;
  uint8_t *gyroscopeValueptr;

  /*Accelerometer Service Value*/
  uint16_t airQualityValue;
  uint8_t *airQualityValueptr;
#endif
  ble_data_struct_t *ble_dataPtr = getBleDataPtr();

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {

    // ******************************************************
    // Events common to both Servers and Clients
    // ******************************************************

    // Boot Event
    case sl_bt_evt_system_boot_id:

      LOG_INFO("System Boot ID\n\r");
#if (DEVICE_IS_BLE_SERVER == 1)
      ble_dataPtr->inflightStatus = false;
      ble_dataPtr->reportDataToDatabase = false;
#endif
      ble_dataPtr->connectionStatus = false;
      ble_dataPtr->buttonStatus = false;
      ble_dataPtr->bondedStatus = false;

      ble_dataPtr->fitnessTrackingEnded = false;

      ble_dataPtr->notificationsDisabled = false;

      /*Initially On TODO:Danger Change*/
      ble_dataPtr->button1Status = sl_bt_gatt_indication;

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&ble_dataPtr->myAddress, &ble_dataPtr->myAddressType);

      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack System Get Identity Address Failed\n\r");
      }
#if (DEVICE_IS_BLE_SERVER == 1)
      // Pad and reverse unique ID to get System ID.
      system_id[0] = ble_dataPtr->myAddress.addr[5];
      system_id[1] = ble_dataPtr->myAddress.addr[4];
      system_id[2] = ble_dataPtr->myAddress.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = ble_dataPtr->myAddress.addr[2];
      system_id[6] = ble_dataPtr->myAddress.addr[1];
      system_id[7] = ble_dataPtr->myAddress.addr[0];


      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Server Write Attribute Value Failed\n\r");
      }

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertisingSetHandle);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Advertiser create set Failed\n\r");
      }

      // Set advertising interval to 250ms.
      sc = sl_bt_advertiser_set_timing(
          advertisingSetHandle, // advertising set handle
          ADVERTISING_MIN_INTERVAL, // min. adv. interval (milliseconds * 1.6)
          ADVERTISING_MAX_INTERVAL, // max. adv. interval (milliseconds * 1.6)
          ADVERTISING_DEFAULT_VALUE,   // adv. duration
          ADVERTISING_DEFAULT_VALUE);  // max. num. adv. events
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Advertiser Set Timing Failed\n\r");
      }
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
          advertisingSetHandle,
          sl_bt_advertiser_general_discoverable,
          sl_bt_advertiser_connectable_scannable);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Advertiser Start Failed\n\r");
      }

      displayInit();
#ifdef LOGGING_BLE_STACK
      LOG_INFO("Event: System Boot ID Completed\n\r");
#endif
      displayPrintf(DISPLAY_ROW_NAME, "Server");

      displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ble_dataPtr->myAddress.addr[0],
                    ble_dataPtr->myAddress.addr[1],
                    ble_dataPtr->myAddress.addr[2],
                    ble_dataPtr->myAddress.addr[3],
                    ble_dataPtr->myAddress.addr[4],
                    ble_dataPtr->myAddress.addr[5]);

      displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A8");
      displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");

      gpioLed0SetOff();
      gpioLed1SetOff();
#else

      // Set passive scanning on 1Mb PHY
      sc = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, 0);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Scanner Set Mode Failed\n\r");
        }

      // Set scan interval and scan window
      sc = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, 80, 40);

      // Set the default connection parameters for subsequent connections
      sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN,
                                                   CONN_INTERVAL_MAX,
                                                   CONN_RESPONDER_LATENCY,
                                                   CONN_TIMEOUT,
                                                   CONN_MIN_CE_LENGTH,
                                                   CONN_MAX_CE_LENGTH);

      // Start scanning - looking for thermometer devices
      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_observation);

      displayInit();
#ifdef LOGGING_BLE_STACK
      LOG_INFO("Event: System Boot ID Completed\n\r");
#endif
      displayPrintf(DISPLAY_ROW_NAME, "Client");

      displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ble_dataPtr->myAddress.addr[0],
                    ble_dataPtr->myAddress.addr[1],
                    ble_dataPtr->myAddress.addr[2],
                    ble_dataPtr->myAddress.addr[3],
                    ble_dataPtr->myAddress.addr[4],
                    ble_dataPtr->myAddress.addr[5]);

      displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Course Project");
      displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");

      sc = sl_bt_sm_configure(0x0F, sl_bt_sm_io_capability_displayyesno);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack SM Configure Failed\n\r");
        }
      sc = sl_bt_sm_delete_bondings();
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Delete Bondings Failed\n\r");
        }

      gpioLed0SetOff();
      gpioLed1SetOff();

#endif

      break;

      // -------------------------------
      // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

      LOG_INFO("Connection Opened Event\n\r");
#if DEVICE_IS_BLE_SERVER
#ifdef LOGGING_BLE_STACK
      LOG_INFO("Event: Connection Opened\n\r");
#endif
      displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
      //displayPrintf(DISPLAY_ROW_TEMPVALUE, "");

      if(evt->data.evt_connection_opened.connection == true)
      {
        ble_dataPtr->connectionStatus = true;
        ble_dataPtr->connectionOpen = evt->data.evt_connection_opened.connection;
      }

      sc = sl_bt_advertiser_stop(advertisingSetHandle);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Advertiser Stop Failed\n\r");
      }

      // Connection Set parameters
      sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection,
                                      CONNECTION_MIN_INTERVAL,                     /*Time = Value x 1.25 ms  Required time = 75ms*/
                                      CONNECTION_MAX_INTERVAL,                     /*Time = Value x 1.25 ms  Required time = 75ms*/
                                      CONNECTION_LATENCY,                      /*latency = Total latency time/ connection interval - 1 : 300ms/75ms - 1 = 3*/
                                      CONNECTION_TIMEOUT,                     /*timeout = (1+latency) * (max_connection_interval)*2 : (1+4)*75*2 = 750ms; value = 750/10 = 75ms */
                                      CONNECTION_MIN_CE_LENGTH,
                                      CONNECTION_MAX_CE_LENGTH);

      /*TODO:Dhiraj New Code Push Button Configuration*/
      sc = sl_bt_sm_configure(0x0F, sl_bt_sm_io_capability_displayyesno);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack SM Configure failed\n\r");
        }
      sc = sl_bt_sm_delete_bondings();
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Delete Bondings failed\n\r");
        }
      // Periodical timer with 1 sec set
      sc = sl_bt_system_set_soft_timer(32768, 1, 0); // Time 1 sec, return handle 1, and false with single shot
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack SM System Soft Timer failed\n\r");
        }





#else

      if(evt->data.evt_connection_opened.connection == true)
      {
        ble_dataPtr->connectionStatus = true;
        ble_dataPtr->connectionOpen = evt->data.evt_connection_opened.connection;
        ble_dataPtr->bleClientEvent = client_connectionOpenedevt;
      }

      displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
      displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ble_dataPtr->serverAddress.addr[0],
                    ble_dataPtr->serverAddress.addr[1],
                    ble_dataPtr->serverAddress.addr[2],
                    ble_dataPtr->serverAddress.addr[3],
                    ble_dataPtr->serverAddress.addr[4],
                    ble_dataPtr->serverAddress.addr[5]);

      /*TODO:Dhiraj Bonding*/
      /*After connection try to form the bonding process*/
      sc = sl_bt_sm_increase_security(ble_data.connectionStatus);
      if (sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack SM Increase Security Failed\n\r");
      }

      // Periodical timer with 1 sec set
      sc = sl_bt_system_set_soft_timer(32768, 1, 0); // Time 1 sec, return handle 1, and false with single shot
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack SM System Soft Timer failed Handle 1\n\r");
        }


      // Periodical timer with 1 sec set
      sc = sl_bt_system_set_soft_timer(131072, 2, 0); // Time 4 sec, return handle 2, and false with single shot
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack SM System Soft Timer failed Handle 2\n\r");
        }

#endif
      break;

      // Connection Closed event
    case sl_bt_evt_connection_closed_id:
      LOG_INFO("Connection Closed Event\n\r");
#if DEVICE_IS_BLE_SERVER
#ifdef LOGGING_BLE_STACK
      LOG_INFO("Event: Connection Closed\n\r", evt->data.evt_connection_closed.reason);
#endif

      displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
      displayPrintf(DISPLAY_ROW_9, "");
      displayPrintf(DISPLAY_ROW_ACTION, "");
      displayPrintf(DISPLAY_ROW_PASSKEY, "");

      if(evt->data.evt_connection_closed.connection == true)
      {
        ble_dataPtr->connectionStatus = false;
        ble_dataPtr->reportDataToDatabase = false;

        ble_dataPtr->buttonStatus = false;
        ble_dataPtr->bondedStatus = false;
        ble_dataPtr->indicationStatus = false;

        /*Assign false to all the variables to be used in the stack*/
        ble_dataPtr->client_config_flags = 0;
        ble_dataPtr->status_flags = 0;
        ble_dataPtr->inflightStatus = false;

      }

      sc = sl_bt_sm_delete_bondings();
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Delete Bondings failed\n\r");
        }



      // Begin Advertising
      sc = sl_bt_advertiser_start(advertisingSetHandle,
                                  sl_bt_advertiser_general_discoverable,
                                  sl_bt_advertiser_connectable_scannable);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Advertiser Start Failed\n\r");
      }

      gpioLed0SetOff();
      gpioLed1SetOff();

#ifdef LOGGING_BLE_STACK
      LOG_INFO("Event: Connection Closed, Restarted Advertising\n\r");
#endif
      break;
#else

      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_observation);

      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Scanner Start Failed\n\r");
        }
      displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
      displayPrintf(DISPLAY_ROW_BTADDR2, "");
      ble_dataPtr->bleClientEvent = client_connectionClosedevt;

      /* Dhiraj Bennadi
       * New Change Danger*/
      ble_dataPtr->connectionStatus = false;

      sc = sl_bt_sm_delete_bondings();
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Delete Bondings Failed\n\r");
        }

      ble_dataPtr->bondedStatus = false;
#endif
      break;


    case sl_bt_evt_connection_parameters_id:

#ifdef TOO_MUCH_LOGGING
      LOG_INFO("Connection Parameters ID\n\r");
      /* Logging Info for Connection Parameters*/
#ifdef LOGGING_BLE_STACK
      LOG_INFO("Client Connection Parameter -> Interval = %d\r\n",(int)((evt->data.evt_connection_parameters.interval)*1.25));
      LOG_INFO("Client Connection Parameter -> Latency = %d\r\n",evt->data.evt_connection_parameters.latency);
      LOG_INFO("Client Connection Parameter -> timeout = %d\r\n",(evt->data.evt_connection_parameters.timeout)*10);
#endif
#endif
      break;

      // ******************************************************
      // Events for Servers/Slaves
      // ******************************************************

      // GATT server characteristic status event
#if DEVICE_IS_BLE_SERVER
    case sl_bt_evt_gatt_server_characteristic_status_id:
      /* Indicates either that a local Client Characteristic Configuration descriptor (CCCD) was changed by the remote GATT client,
       * or that a confirmation from the remote GATT client was received upon a successful reception of the indication
       */
      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
        {
          if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            {
              switch(evt->data.evt_gatt_server_characteristic_status.client_config_flags)
              {
                case sl_bt_gatt_server_indication:
                  ble_dataPtr->connectionOpen = evt->data.evt_gatt_server_characteristic_status.connection;
                  if(evt->data.evt_gatt_server_characteristic_status.connection == true)
                    {
                      ble_dataPtr->reportDataToDatabase = true;
                      gpioLed0SetOn();
                    }
                  else
                    {
                      ble_dataPtr->reportDataToDatabase = false;
                      ble_dataPtr->inflightStatus = false;
                      gpioLed0SetOff();
                    }
                  break;

                case sl_bt_gatt_server_disable:
                  ble_dataPtr->reportDataToDatabase = false;
                  ble_dataPtr->inflightStatus = false;
                  displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
                  gpioLed0SetOff();
                  break;

                default:
                  break;
              }
            }

          if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
          {
              ble_dataPtr->inflightStatus = false;
          }
        }

      /*Button GATT Service*/
      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state)
        {

          ble_dataPtr->client_config_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;
          ble_dataPtr->status_flags = evt->data.evt_gatt_server_characteristic_status.status_flags;

          if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            {
              switch(evt->data.evt_gatt_server_characteristic_status.client_config_flags)
              {
                case sl_bt_gatt_server_indication:
                  ble_dataPtr->indicationStatus = true;
                  gpioLed1SetOn();
#ifdef LOGGING_ENABLE
                  LOG_INFO("Button Indication Status With Event Indication= %d\n\r", ble_dataPtr->indicationStatus);
#endif
                  break;

                case sl_bt_gatt_server_disable:
                  ble_dataPtr->indicationStatus = false;
                  ble_dataPtr->inflightStatus = false;
                  gpioLed1SetOff();
#ifdef LOGGING_ENABLE
                  LOG_INFO("Button Indication Status With Server Disable= %d\n\r", ble_dataPtr->indicationStatus);
#endif
                  break;

                default:
                  break;
              }

              if(evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
              {
                  ble_dataPtr->inflightStatus = false;
                  gpioLed1SetOff();
              }
            }
        }
         break;
#endif

    case sl_bt_evt_system_soft_timer_id:

      get_queue_status(&buttonIndicationNodesStats);

      if(ble_dataPtr->fitnessTrackingEnded == true)
        {

          ble_dataPtr->fitnessTrackingEnded = false;

          LOG_INFO("Fitness ST = %d\n\r", ble_dataPtr->fitnessTrackingStartTime);
          LOG_INFO("Fitness ET = %d\n\r", ble_dataPtr->fitnessTrackingEndTime);

          ble_dataPtr->fitnessTrackingDuration =
              (ble_dataPtr->fitnessTrackingEndTime - ble_dataPtr->fitnessTrackingStartTime) / 1000;

          if(ble_dataPtr->fitnessTrackingDuration > 30)
            {
              ble_dataPtr->fitnessResult = 1;
            }
          else
            {
              ble_dataPtr->fitnessResult = 2;
            }

          LOG_INFO("Fitness Delta = %d\n\r", ble_dataPtr->fitnessTrackingDuration);

          displayPrintf(DISPLAY_ROW_PASSKEY, "Duration = %d s" , ble_dataPtr->fitnessTrackingDuration);
          displayPrintf(DISPLAY_ROW_ACTION, "Number of Steps = %d", ble_dataPtr->stepCounter);
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
          displayPrintf(DISPLAY_ROW_8, "");
          displayPrintf(DISPLAY_ROW_CONNECTION, "");

        }



      if(evt->data.evt_system_soft_timer.handle == 0)
        {
          displayUpdate();

          if(ble_dataPtr->fitnessTrackingEnded == true)
            {
              //fitnessTrackingEndedIndication();

            }
        }

      if(evt->data.evt_system_soft_timer.handle == 1)
        {

          if((buttonIndicationNodesStats.isCBEmpty == false))
            {
              get_queue_status(&buttonIndicationNodesStats);
              LOG_INFO("Soft Timer Handle 1 Hit After Enqueuing\n\r");

              circularBuffer_t tempBuffer;

              read_queue(&tempBuffer);

              LOG_INFO("Circular Buffer Empty = %d\n\r", buttonIndicationNodesStats.isCBEmpty);

              sc = sl_bt_gatt_set_characteristic_notification(ble_dataPtr->connectionStatus,
                                                              tempBuffer.charHandle,
                                                              tempBuffer.indicationStatus);
              if (!(sc == SL_STATUS_OK))
                {
                  LOG_ERROR("GATT Set Characteristic Notification Failed = %X\r\n", sc);
                }

            }

          if(ble_dataPtr->fitnessResult == 1)
            {
              fitnessTrackingEndedIndication();
              displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
              displayPrintf(DISPLAY_ROW_8, "");
              displayPrintf(DISPLAY_ROW_CONNECTION, "");
            }
        }

      if(evt->data.evt_system_soft_timer.handle == 2)
        {
          if(ble_dataPtr->fitnessResult == 2)
            {
              fitnessTrackingEndedIndication();
              displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
              displayPrintf(DISPLAY_ROW_8, "");
              displayPrintf(DISPLAY_ROW_CONNECTION, "");
            }
        }
#if DEVICE_IS_BLE_SERVER
      if(evt->data.evt_system_soft_timer.handle == 1)
        {
          if((buttonIndicationNodesStats.isCBEmpty == false) && (ble_dataPtr->inflightStatus == false))
            {
              read_queue(&buttonIndicationNode);

              sc = sl_bt_gatt_server_write_attribute_value(buttonIndicationNode.charHandle,
                                                           NO_OFFSET,
                                                           buttonIndicationNode.bufferLength,
                                                           buttonIndicationNode.buffer);

              if(sc != SL_STATUS_OK)
                {
                  LOG_ERROR("BT:Stack Server Write Attribute failed\n\r");
                }

              sc = sl_bt_gatt_server_send_indication(ble_dataPtr->connectionOpen,
                                                     buttonIndicationNode.charHandle,
                                                     buttonIndicationNode.bufferLength,
                                                     buttonIndicationNode.buffer);

              if(sc != SL_STATUS_OK)
                {
                  LOG_ERROR("BT:Stack Server Send Indication failed\n\r");
                }

              ble_dataPtr->inflightStatus = true;

            }
        }
#endif
      break;


      /*Only Server Event*/
    case sl_bt_evt_sm_confirm_bonding_id:
      LOG_INFO("SM Confirm Bonding ID\n\r");

#ifdef LOGGING_ENABLE
      LOG_INFO("Bonding Process Completed\n\r");
#endif
      sc = sl_bt_sm_bonding_confirm(ble_dataPtr->connectionOpen, ACCEPT_BONDING_REQUEST);

      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("BT:Stack Bonding Confirmation Failed\n\r");
        }

      break;

    case sl_bt_evt_sm_confirm_passkey_id:
      LOG_INFO("SM Confirm Passkey ID\n\r");
      passKey = evt->data.evt_sm_confirm_passkey.passkey;
      displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
      displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %4lu", passKey);
      break;

    case sl_bt_evt_system_external_signal_id:

#if DEVICE_IS_BLE_SERVER
            if(evt->data.evt_system_external_signal.extsignals == evtGPIO_IRQ){

                if(ble_dataPtr->buttonStatus == 0){
                    sc = sl_bt_sm_passkey_confirm(ble_dataPtr->connectionStatus, 1);
                    ble_dataPtr->buttonStatus = 1;
                    ble_dataPtr->bondedStatus = 1;
                    app_assert_status(sc);
#ifdef LOGGING_ENABLE
                    LOG_INFO("Passkey confirm\r\n");
#endif

                }

                ble_dataPtr->buttonValue = GPIO_PinInGet(PB0_port, PB0_pin);

                if(ble_dataPtr->buttonValue == BUTTON_RELEASED){
                    displayPrintf(DISPLAY_ROW_9, "Button Released");
                    sc =  sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                                  NO_OFFSET,
                                                                  1,
                                                                  (const uint8_t*)&ble_dataPtr->buttonValue);
                    if(sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("BT:Stack Write Attribute Failed\n\r");
                      }

                    sc = sl_bt_gatt_server_send_indication(ble_dataPtr->connectionOpen,
                                                                          gattdb_button_state,
                                                                          1,
                                                                          (const uint8_t*)&ble_dataPtr->buttonValue);
                    if(sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("BT:Stack Send Indication Failed\n\r");
                      }

                }
                else if(ble_dataPtr->buttonValue == BUTTON_PRESSED)
                  {
                    displayPrintf(DISPLAY_ROW_9, "Button Pressed");
                    sc =  sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                                  NO_OFFSET,
                                                                  1,
                                                                  (const uint8_t*)&ble_dataPtr->buttonValue);
                    if(sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("BT:Stack Write Attribute Failed\n\r");
                      }
                    sc = sl_bt_gatt_server_send_indication(ble_dataPtr->connectionOpen,
                                                                         gattdb_button_state,
                                                                         1,
                                                                         (const uint8_t*)&ble_dataPtr->buttonValue);
                    if(sc != SL_STATUS_OK)
                      {
                        LOG_ERROR("BT:Stack Send Indication Failed\n\r");
                      }

                }
                // For button status value
                uint8_t *p = (uint8_t*)&ble_dataPtr->buttonValue;
                UINT8_TO_BITSTREAM(p, ble_dataPtr->buttonValue);


                if((evt->data.evt_connection_opened.connection == 1) && (ble_dataPtr->bondedStatus == 1)
                    && (ble_dataPtr->indicationStatus == 1) && (ble_dataPtr->inflightStatus == 0)){

                    if (!(sc == SL_STATUS_OK)) {
                        LOG_ERROR("Error in button_state write attribute value = %X", sc);
                    }

                    if (!(sc == SL_STATUS_OK)) {
                        LOG_ERROR("Error in button_state send_indication value = %X", sc);
                    }

                }
                // For Inflight condition, sending data to buffer
                else if((evt->data.evt_connection_opened.connection == 1) && (ble_dataPtr->bondedStatus == 1) &&
                    (ble_dataPtr->indicationStatus == 1) && (ble_dataPtr->inflightStatus == 1)){

                    buttonIndicationNode.charHandle = gattdb_button_state;
                    buttonIndicationNode.bufferLength = sizeof(char) & 0x000000FF;
                    buttonIndicationNode.buffer[0] = ble_dataPtr->buttonValue;
                    buttonIndicationNode.buffer[1] = 0;
                    buttonIndicationNode.buffer[2] = 0;
                    buttonIndicationNode.buffer[3] = 0;
                    buttonIndicationNode.buffer[4] = 0;

                    bool returnVal = write_queue(buttonIndicationNode);

                    if(returnVal != 0)
                      {
                        LOG_ERROR("Enqueue Failed\n\r");
                      }
                }

            }
#else


          if(evt->data.evt_system_external_signal.extsignals == evtGPIO_IRQ)
            {
              LOG_INFO("SM External Signal ID----------> GPIO 0\n\r");

              /*TODO:Dhiraj Bonding Process*/
              if(ble_dataPtr->buttonStatus == 0)
                {
                  sc = sl_bt_sm_passkey_confirm(ble_dataPtr->connectionStatus, 1);
                  if(sc != SL_STATUS_OK)
                    {
                      LOG_ERROR("BT:Stack SM Passkey Confirm Error\n\r");
                    }
                  ble_dataPtr->buttonStatus = 1;
                  ble_dataPtr->bondedStatus = 1;
#ifdef LOGGING_ENABLE
                  LOG_INFO("Passkey confirm\r\n");
#endif

                }




            }

#ifdef NOT_NEEDED_FOR_BONDING
              if(ble_dataPtr->sequenceInitiated == true)
                {
                  if(ble_dataPtr->buttonStatus == 0)
                    {
                      sc = sl_bt_sm_passkey_confirm(ble_dataPtr->connectionStatus, 1);
                      if(sc != SL_STATUS_OK)
                        {
                          LOG_ERROR("BT:Stack SM Passkey Confirm Error\n\r");
                        }
                      ble_dataPtr->buttonStatus = 1;
                      ble_dataPtr->bondedStatus = 1;
#ifdef LOGGING_ENABLE
                      LOG_INFO("Passkey confirm\r\n");
#endif

                    }

                }

            }
          else if(evt->data.evt_system_external_signal.extsignals == evtGPIO1_IRQ)
            {
              LOG_INFO("SM External Signal ID----------> GPIO 1\n\r");
              ble_dataPtr->sequenceInitiated = true;

              if(GPIO_PinInGet(gpioPortF, PB0_pin) == 1)
                {
                  LOG_INFO("ble_data.openedConnection = %d\r\n", ble_dataPtr->connectionStatus);
                  LOG_INFO("ble_data.button_characteristic_handle = %d\r\n", ble_dataPtr->buttonCharacteristicHandle);

                  sc = sl_bt_gatt_read_characteristic_value(ble_dataPtr->connectionStatus,  //Connection handle
                                                            ble_dataPtr->buttonCharacteristicHandle); // characteristic stored for push button
                  if (!(sc == SL_STATUS_OK))
                    {
                      LOG_ERROR("Error in button_state PB1 read characteristic 1 = %X\r\n", sc);
                    }
                }
              // Checking the interrupt for PB1 and PB0 Pressed
              else if(GPIO_PinInGet(gpioPortF, PB0_pin) == 0)
                {
                  if(ble_dataPtr->buttonPB1Status == 0)
                    {
                      ble_dataPtr->buttonIndicationStatus = (ble_dataPtr->buttonIndicationStatus)^2; // XOR for 0x00 and 0x02 value to be passed as an argument

                      LOG_INFO("ble_data.indicate = %d\r\n", ble_dataPtr->buttonIndicationStatus);

                      sc = sl_bt_gatt_set_characteristic_notification(ble_dataPtr->connectionStatus,
                                                                      ble_dataPtr->buttonCharacteristicHandle,
                                                                      (ble_dataPtr->buttonIndicationStatus));
                      if (!(sc == SL_STATUS_OK))
                        {
                          LOG_ERROR("Error in button_state PB1 set characteristic 0 = %X\r\n", sc);
                        }
                      ble_dataPtr->buttonPB1Status = 1;
                    }
                  else
                    {
                      ble_dataPtr->buttonPB1Status = 0;
                    }
                }
            }
#endif

          /*TODO:Dhiraj Bennadi*/
          /*Disable Notifications*/
          if(evt->data.evt_system_external_signal.extsignals == evtGPIO1_IRQ)
            {

              LOG_INFO("SM External Signal ID----------> GPIO 1\n\r");

              if(ble_dataPtr->notificationsDisabled == false)
                {

                  ble_dataPtr->button1Status ^= 2;

                  circularBuffer_t tempBuffer[3];

                  tempBuffer[0].charHandle = ble_dataPtr->accelerometerCharactersticHandle;
                  tempBuffer[0].bufferLength = 2;
                  tempBuffer[0].indicationStatus = ble_dataPtr->button1Status;
                  tempBuffer[0].buffer[0] = 0;
                  tempBuffer[0].buffer[1] = 0;


                  tempBuffer[1].charHandle = ble_dataPtr->GyroscopeCharactersticHandle;
                  tempBuffer[1].bufferLength = 2;
                  tempBuffer[1].indicationStatus = ble_dataPtr->button1Status;
                  tempBuffer[1].buffer[0] = 0;
                  tempBuffer[1].buffer[1] = 0;

                  tempBuffer[2].charHandle = ble_dataPtr->AirQualityCharactersticHandle;
                  tempBuffer[2].bufferLength = 2;
                  tempBuffer[2].indicationStatus = ble_dataPtr->button1Status;
                  tempBuffer[2].buffer[0] = 0;
                  tempBuffer[2].buffer[1] = 0;
                  write_queue(tempBuffer[0]);
                  write_queue(tempBuffer[1]);
                  write_queue(tempBuffer[2]);



                  get_queue_status(&buttonIndicationNodesStats);


                  ble_dataPtr->notificationsDisabled = true;

                  if(ble_dataPtr->endTimeRecorded == false)
                    {
                      ble_dataPtr->endTimeRecorded = true;

                      ble_dataPtr->fitnessTrackingEndTime = loggerGetTimestamp();
                    }

                  ble_dataPtr->fitnessTrackingEnded = true;

//                  for(int i =0; i < 10000; i++)
//                    {
//                      ;
//                    }
//
//
//
//                  sc = sl_bt_gatt_set_characteristic_notification(ble_dataPtr->connectionStatus,
//                                                                  ble_dataPtr->accelerometerCharactersticHandle,
//                                                                  ble_dataPtr->button1Status);
//                  if (!(sc == SL_STATUS_OK))
//                    {
//                      LOG_ERROR("Error in button_state PB1 set characteristic 0 = %X\r\n", sc);
//                    }


                }
              else
                {
                  ble_dataPtr->notificationsDisabled = false;
                }


//
//              if(ble_dataPtr->notificationsDisabled == false)
//                {
//
//
//
//                  ble_dataPtr->notificationsDisabled = true;
//                  ble_dataPtr->button1Status ^= 2;
//                }

            }


#endif

          break;

          /*This event completes Bonding*/
    case sl_bt_evt_sm_bonded_id:
      LOG_INFO("SM Bonded ID\n\r");
#ifdef LOGGING_ENABLE
      LOG_INFO("Bonding Successful\n\r");
#endif
      displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
      displayPrintf(DISPLAY_ROW_ACTION, "");
      displayPrintf(DISPLAY_ROW_PASSKEY, "");

      ble_dataPtr->bondedStatus = true;

      break;

    case  sl_bt_evt_sm_bonding_failed_id:
      LOG_INFO("SM Bonding Failed ID\n\r");
      LOG_INFO("Bonding failed, reason 0x%2X\r\n", evt->data.evt_sm_bonding_failed.reason);

      //LOG_ERROR("Bonding Failed\n\r");
#ifdef LOGGING_ENABLE
        LOG_INFO("Bonding failed, reason 0x%2X\r\n", evt->data.evt_sm_bonding_failed.reason);
#endif
        break;

    case sl_bt_evt_gatt_server_indication_timeout_id:
      //LOG_INFO("SM Confirm Bonding ID\n\r");
      // When the indication times out we will log it and then reset the system
      LOG_ERROR("Sever Indication Time Out. Connection Handle: %d\n\nReconnect to proceed",evt->data.evt_gatt_server_indication_timeout.connection);
      sl_bt_system_reset(sl_bt_system_boot_mode_normal);
      break;

#if DEVICE_IS_BLE_SERVER
#else
    case sl_bt_evt_scanner_scan_report_id:
      //LOG_INFO("Scanner Scan Report ID\n\r");
      ble_dataPtr->serverAddress = SERVER_BT_ADDRESS;

      /*LOG Info*/
      /*Compare Address*/

      if(!memcmp(evt->data.evt_scanner_scan_report.address.addr,
                 ble_dataPtr->serverAddress.addr , sizeof(bd_addr)))
        {

          sc = sl_bt_scanner_stop();
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack BT Scanner Stop Error\n\r");
            }
          sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                   evt->data.evt_scanner_scan_report.address_type,
                                   sl_bt_gap_1m_phy,
                                   NULL);
          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BL:Stack BT Connection Open Error\n\r");
            }
        }
      break;

    case sl_bt_evt_gatt_service_id:
      LOG_INFO("GATT Service ID\n\r");
      if(!memcmp(evt->data.evt_gatt_service.uuid.data, thermo_service , sizeof(thermo_service)))
        {
          ble_dataPtr->tempServiceHandle = evt->data.evt_gatt_service.service;
        }

      if(!memcmp(evt->data.evt_gatt_service.uuid.data, &button_service , sizeof(button_service)))
        {
          ble_dataPtr->buttonServiceHandle = evt->data.evt_gatt_service.service;
        }

      /*Accelerometer Service*/
      if(!memcmp(evt->data.evt_gatt_service.uuid.data, accelerometer_service , sizeof(accelerometer_service)))
        {
          ble_dataPtr->AccelerometerServiceHandle = evt->data.evt_gatt_service.service;
        }
      LOG_INFO("Accelerometer Service Handle = %x \n\r", ble_dataPtr->AccelerometerServiceHandle);

      /*Gyroscope Service*/
      if(!memcmp(evt->data.evt_gatt_service.uuid.data, gyroscope_service , sizeof(gyroscope_service)))
        {
          ble_dataPtr->GyroscopeServiceHandle = evt->data.evt_gatt_service.service;
        }
      LOG_INFO("Gyroscope Service Handle = %x \n\r", ble_dataPtr->GyroscopeServiceHandle);

      /*Gyroscope Service*/
      if(!memcmp(evt->data.evt_gatt_service.uuid.data, airQualityIndex_service , sizeof(airQualityIndex_service)))
        {
          ble_dataPtr->AirQualityServiceHandle = evt->data.evt_gatt_service.service;
        }
      LOG_INFO("Air Quality Service Handle = %x \n\r", ble_dataPtr->AirQualityServiceHandle);

      break;

    case sl_bt_evt_gatt_characteristic_id:
      LOG_INFO("GATT Characteristic ID\n\r");
      if(!memcmp(evt->data.evt_gatt_characteristic.uuid.data, thermo_char , sizeof(thermo_char)))
        {
          ble_dataPtr->tempCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
        }

      if(!memcmp(evt->data.evt_gatt_characteristic.uuid.data, &button_char , sizeof(button_char)))
        {
          ble_dataPtr->buttonCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
        }

      /*Accelerometer Service*/
      if(!memcmp(evt->data.evt_gatt_characteristic.uuid.data, accelerometer_char , sizeof(accelerometer_char)))
        {
          ble_dataPtr->accelerometerCharactersticHandle = evt->data.evt_gatt_characteristic.characteristic;
        }

      /*Gyroscope Service*/
      if(!memcmp(evt->data.evt_gatt_characteristic.uuid.data, gyroscope_char , sizeof(gyroscope_char)))
        {
          ble_dataPtr->GyroscopeCharactersticHandle = evt->data.evt_gatt_characteristic.characteristic;
        }

      /*Gyroscope Service*/
      if(!memcmp(evt->data.evt_gatt_characteristic.uuid.data, airQualityIndex_char , sizeof(airQualityIndex_char)))
        {
          ble_dataPtr->AirQualityCharactersticHandle = evt->data.evt_gatt_characteristic.characteristic;
        }
      break;

    case sl_bt_evt_gatt_procedure_completed_id:
      LOG_INFO("SM GATT Procedure Completed ID\n\r");
      if(evt->data.evt_gatt_procedure_completed.result == 0)
        {
          ble_dataPtr->bleClientEvent = client_gattServiceCompletedevt;
        }
#ifdef NOT_NEEDED_FOR_BONDING
      else if(evt->data.evt_gatt_procedure_completed.result ==
          (sl_status_t)SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION)
        {
          sc = sl_bt_sm_increase_security(ble_data.connectionStatus);
          if (sc != SL_STATUS_OK){
              LOG_ERROR("BT:Stack SM Increase Security Failed\n\r");
           }
        }
#endif
      break;

    case sl_bt_evt_gatt_characteristic_value_id:
      /*TODO:Dhiraj Commenting the below line to avoid excess data*/
      LOG_INFO("GATT Characteristic Value ID\n\r");

      if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
          && (evt->data.evt_gatt_characteristic_value.characteristic == ble_dataPtr->tempCharacteristicHandle))
        {
          ble_dataPtr->bleClientEvent = client_characteristicValueIndication;

          tempValueptr = &(evt->data.evt_gatt_characteristic_value.value.data[0]);

          tempValueInt = FLOAT_TO_INT32(tempValueptr);
          displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temperature = %d", tempValueInt);

          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BT:Stack GATT Send Characteristic Confirmation Failed\n\r");
            }

        }
      else if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
          && (evt->data.evt_gatt_characteristic_value.characteristic == ble_dataPtr->accelerometerCharactersticHandle))
        {

          if(ble_dataPtr->startTimeRecorded == false)
            {
              ble_dataPtr->startTimeRecorded = true;

              ble_dataPtr->fitnessTrackingStartTime = loggerGetTimestamp();
            }
          ble_dataPtr->bleClientEvent = client_characteristicValueIndication;

          accelerometerValueptr = &(evt->data.evt_gatt_characteristic_value.value.data[0]);

          acclerometerValue = (accelerometerValueptr[0] << 8) | (accelerometerValueptr[1]);

          //LOG_INFO("Accelerometer Value = %d\n\r", accelerometerValueptr[0]);
          //LOG_INFO("Accelerometer Value = %d\n\r", accelerometerValueptr[1]);
          //LOG_INFO("Accelerometer Value = %d\n\r", acclerometerValue);

          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BT:Stack GATT Send Characteristic Confirmation Failed\n\r");
            }

          if(acclerometerValue > 2)
            {
              ble_dataPtr->stepCounter += 1;
            }

          displayPrintf(DISPLAY_ROW_CONNECTION, "H I Accel");
          displayPrintf(DISPLAY_ROW_8, "A Tilt = %d", acclerometerValue);
        }
      else
        {
          ;
        }


      /************************Gyroscope***************************/
      if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
          && (evt->data.evt_gatt_characteristic_value.characteristic == ble_dataPtr->GyroscopeCharactersticHandle))
        {
          ble_dataPtr->bleClientEvent = client_characteristicValueIndication;

          gyroscopeValueptr = &(evt->data.evt_gatt_characteristic_value.value.data[0]);

          gyroscopeValue = (gyroscopeValueptr[0] << 8) | (gyroscopeValueptr[1]);

          //LOG_INFO("Gyroscope Value = %d\n\r", gyroscopeValueptr[0]);
          //LOG_INFO("Gyroscope Value = %d\n\r", gyroscopeValueptr[1]);
          //LOG_INFO("Accelerometer Value = %d\n\r", acclerometerValue);

          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BT:Stack GATT Send Characteristic Confirmation Failed\n\r");
            }

          displayPrintf(DISPLAY_ROW_CONNECTION, "H I Gyro");
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Gyroscope = %d", gyroscopeValue);

        }
      else
        {
          //displayPrintf(DISPLAY_ROW_CONNECTION, "Disabled Indications");
          //displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
          //displayPrintf(DISPLAY_ROW_8, "");
        }

      /***********************Air Quality*****************************/
      if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
          && (evt->data.evt_gatt_characteristic_value.characteristic == ble_dataPtr->AirQualityCharactersticHandle))
        {
          ble_dataPtr->bleClientEvent = client_characteristicValueIndication;

          airQualityValueptr = &(evt->data.evt_gatt_characteristic_value.value.data[0]);

          airQualityValue = (airQualityValueptr[0] << 8) | (airQualityValueptr[1]);

          //LOG_INFO("Gyroscope Value = %d\n\r", gyroscopeValueptr[0]);
          //LOG_INFO("Gyroscope Value = %d\n\r", gyroscopeValueptr[1]);
          //LOG_INFO("Accelerometer Value = %d\n\r", acclerometerValue);

          sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);

          if(sc != SL_STATUS_OK)
            {
              LOG_INFO("BT:Stack GATT Send Characteristic Confirmation Failed\n\r");
            }

          displayPrintf(DISPLAY_ROW_CONNECTION, "H I Air Quality");
          displayPrintf(DISPLAY_ROW_11, "Air Quality = %d", airQualityValue);

        }
      else
        {
          //displayPrintf(DISPLAY_ROW_CONNECTION, "Disabled Indications");
          //displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
          //displayPrintf(DISPLAY_ROW_8, "");
        }


#ifdef DISABLING_BUTTON_SERVICE
      if(((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)||
          (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response))&&
        (evt->data.evt_gatt_characteristic_value.characteristic == ble_dataPtr->buttonCharacteristicHandle)){
          ble_dataPtr->bleClientEvent = client_characteristicValueIndication;

          if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
            {
              sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);


              if(sc != SL_STATUS_OK)
                {
                  LOG_ERROR("BT:Stack GATT Send Characteristic Confirmation Failed\n\r");
                }
            }

        uint8_t button_pressed_client = (evt->data.evt_gatt_characteristic_value.value.data[0]);
        //LOG_INFO("button pressed = %u\r\n", button_pressed_client);
        if(button_pressed_client == 1){
          displayPrintf(DISPLAY_ROW_9, "Button Released");
          }
          else if(button_pressed_client == 0){
          displayPrintf(DISPLAY_ROW_9, "Button Pressed");
          }

        }

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                   0,
                                                   1,
                                                   &ble_dataPtr->buttonStatusValue);

      sl_bt_gatt_server_send_indication(ble_dataPtr->connectionStatus,
                                        gattdb_button_state,
                                        1,
                                        &ble_dataPtr->buttonStatusValue);
#endif
      break;
#endif
    /*Default Case*/
    default:
      break;


  }
}
/***************************************************************************//**
  void updateTempValueInBleDatabase(float temp_c);
 * Description: void updateTempValueInBleDatabase is called from I2C.c while everytime the
 *              I2C reports the temperature data. The condition for GATT DB transfer
 *              in case sl_bt_evt_gatt_server_characteristic_status_id is obtained
 *              and checked for the temperature process as per functionality.
 ******************************************************************************/
void updateTempValueInBleDatabase(float temp_c)
{
  sl_status_t sc;

  ble_data_struct_t *ble_dataPtr = getBleDataPtr();

  if(ble_dataPtr->reportDataToDatabase == true)
  {

      /*Temperature transfer using Gatt server write attribute*/
      uint8_t htm_temperature_buffer[5];
      uint8_t *p = htm_temperature_buffer;
      uint32_t htm_temperature_flt;
      uint8_t flags = 0x00;

      UINT8_TO_BITSTREAM(p, flags);

      /* Convert temperature to bitstream and place it in the htm_temperature_buffer*/
      htm_temperature_flt = UINT32_TO_FLOAT(temp_c*1000, -3);


      UINT32_TO_BITSTREAM(p, htm_temperature_flt);
      sc = sl_bt_gatt_server_write_attribute_value(gattdb_temperature_measurement,
                                                   0,
                                                   sizeof(htm_temperature_buffer),
                                                   htm_temperature_buffer);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Server Write Attribute Value Failed\n\r");
      }
      //app_assert_status(sc);

      sc = sl_bt_gatt_server_send_indication(ble_dataPtr->connectionOpen,
                                             gattdb_temperature_measurement,
                                             5,
                                             htm_temperature_buffer);
      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("BT:Stack Server Send Indication Failed\n\r");
      }

      ble_dataPtr->inflightStatus = true;

      displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temperature = %d" , (int)temp_c);
  }
  else
  {
      //No Action
  }
}


#if DEVICE_IS_BLE_SERVER
#else
// -----------------------------------------------
// Private function, original from Dan Walkes. I fixed a sign extension bug.
// We'll need this for Client A7 assignment to convert health thermometer
// indications back to an integer. Convert IEEE-11073 32-bit float to signed integer.
// -----------------------------------------------
static int32_t FLOAT_TO_INT32(const uint8_t *value_start_little_endian)
{
  uint8_t signByte = 0;
  int32_t mantissa;
  int32_t returnValue = 0;
  // input data format is:
  // [0] = flags byte
  // [3][2][1] = mantissa (2's complement)
  // [4] = exponent (2's complement)
  // BT value_start_little_endian[0] has the flags byte
  int8_t exponent = (int8_t)value_start_little_endian[4];
  // sign extend the mantissa value if the mantissa is negative
  if (value_start_little_endian[3] & 0x80) { // msb of [3] is the sign of the mantissa
      signByte = 0xFF;
  }
  mantissa = (int32_t) (value_start_little_endian[1] << 0) |
      (value_start_little_endian[2] << 8) |
      (value_start_little_endian[3] << 16) |
      (signByte << 24) ;
  // value = 10^exponent * mantissa, pow() returns a double type

  returnValue = (int32_t) (pow(10, exponent) * mantissa);
  return returnValue;
} // FLOAT_TO_INT32
#endif

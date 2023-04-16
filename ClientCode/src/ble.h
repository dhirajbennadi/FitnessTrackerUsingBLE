/*
 * ble.h
 *
 *  Created on: April 28th, 2022
 *  Author: Dhiraj Bennadi
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include "sl_bt_api.h"

#include "gatt_db.h"
#include "app_log.h"
#include "app_assert.h"

#include "stdbool.h"
#include "scheduler.h"

// Health Thermometer service UUID defined by Bluetooth SIG
extern const uint8_t thermo_service[2];
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
extern const uint8_t thermo_char[2];

extern const uint8_t button_service[16];
// Button Press characteristic UUID
extern const uint8_t button_char[16];

extern const uint8_t accelerometer_service[16];

extern const uint8_t accelerometer_char[16];

extern const uint8_t gyroscope_service[16];

extern const uint8_t gyroscope_char[16];

extern const uint8_t airQualityIndex_service[16];

extern const uint8_t airQualityIndex_char[16];


void handle_ble_event(sl_bt_msg_t *evt);
void updateTempValueInBleDatabase(float temp_c);

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }
#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
*(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }
#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// file = ble.h
// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
  // values that are common to servers and clients
  bd_addr myAddress;
  uint8_t myAddressType;
  // values unique for server
  uint8_t advertisingSetHandle;
  bool connectionStatus;
  bool reportDataToDatabase;
  bool inflightStatus;
  uint8_t connectionOpen;

  /*Assignment 8*/
  uint8_t client_config_flags;
  uint8_t status_flags;

  bool buttonStatus;
  bool bondedStatus;
  bool indicationStatus;
  uint32_t buttonValue;
  // values unique for client

  clientEvents bleClientEvent;
  bd_addr serverAddress;
  uint32_t tempServiceHandle;
  uint16_t tempCharacteristicHandle;

  uint32_t buttonServiceHandle;
  uint16_t buttonCharacteristicHandle;

  uint8_t buttonStatusValue;
  bool buttonPB1Status;
  uint8_t buttonIndicationStatus;

  bool sequenceInitiated;

  /*Accelerometer Service*/
  uint32_t AccelerometerServiceHandle;
  uint16_t accelerometerCharactersticHandle;

  /*Gyroscope Service*/
  uint32_t GyroscopeServiceHandle;
  uint16_t GyroscopeCharactersticHandle;

  /*Air Quality Index Service*/
  uint32_t AirQualityServiceHandle;
  uint16_t AirQualityCharactersticHandle;

  uint8_t button1Status;
  bool notificationsDisabled;

  bool fitnessTrackingEnded;
  bool fitnessTrackingPaused;


  uint32_t fitnessTrackingDuration;
  uint32_t fitnessTrackingStartTime;
  uint32_t fitnessTrackingEndTime;

  bool startTimeRecorded;
  bool endTimeRecorded;

  uint8_t fitnessResult;

  uint16_t stepCounter;

} ble_data_struct_t;
// function prototypes

ble_data_struct_t* getBleDataPtr(void);


// connection parameters
#define CONN_INTERVAL_MIN             60   //100ms
#define CONN_INTERVAL_MAX             60   //100ms
#define CONN_RESPONDER_LATENCY        3    //no latency
#define CONN_TIMEOUT                  81  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            4

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

#endif /* SRC_BLE_H_ */

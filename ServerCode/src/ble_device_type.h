/*
 * ble_device_type.h
 *
 *  Created on: Feb 16, 2019
 *      Author: dan walkes
 *
 *      Editor: Mar 14, 2021, Dave Sluiter
 *      Change: Added some vertical white space and bd_addr array indices
 */

#ifndef SRC_BLE_DEVICE_TYPE_H_
#define SRC_BLE_DEVICE_TYPE_H_
#include <stdbool.h>

/*
 * Students:
 * Set to 1 to configure this build as a BLE server.
 * Set to 0 to configure as a BLE client
 */
#define DEVICE_IS_BLE_SERVER 1


// For your Bluetooth Client implementations.
// Set this #define to the bd_addr of the Gecko that will be your Server.
//                   bd_addr  [0]   [1]   [2]   [3]   [4]   [5] <- array indices
//#define SERVER_BT_ADDRESS (bd_addr) { .addr = { 0x2D, 0x03, 0x92, 0x27, 0xFD, 0x84 } }

//#define SERVER_BT_ADDRESS (bd_addr) { .addr = { 0x84, 0xFD, 0x27, 0x92, 0x03, 0x2D } }

/*Board 255 Address*/
#define SERVER_BT_ADDRESS (bd_addr) { .addr = { 0x2D, 0x03, 0x92, 0x27, 0xFD, 0x84 } }

/*Board 124 Address*/
#define SERVER_BT_ADDRESS2 (bd_addr) { .addr = { 0x20, 0x2C, 0x61, 0xCC, 0xCC, 0xCC } }
//#define SERVER_BT_ADDRESS4 (bd_addr) { .addr = { 0xCC, 0xCC, 0xCC, 0x61, 0x2C, 0x20 } }

#if DEVICE_IS_BLE_SERVER

#define BUILD_INCLUDES_BLE_SERVER 1
#define BUILD_INCLUDES_BLE_CLIENT 0
#define BLE_DEVICE_TYPE_STRING "Server"
static inline bool IsServerDevice() { return true; }
static inline bool IsClientDevice() { return false; }

#else

#define BUILD_INCLUDES_BLE_SERVER 0
#define BUILD_INCLUDES_BLE_CLIENT 1
#define BLE_DEVICE_TYPE_STRING "Client"
static inline bool IsClientDevice() { return true;}
static inline bool IsServerDevice() { return false; }

#endif

#endif /* SRC_BLE_DEVICE_TYPE_H_ */

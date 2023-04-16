/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Date:        08-07-2021
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 ******************************************************************************/


// *************************************************
// Students: It is OK to modify this file.
//           Make edits appropriate for each
//           assignment.
// *************************************************


#ifndef APP_H
#define APP_H

/*Energy Mode 0 - Run Mode*/
#define EM0 0
/*Energy Mode 1 - Sleep Mode*/
#define EM1 1
/*Energy Mode 2 - Deep Sleep Mode*/
#define EM2 2
/*Energy Mode 3 - Stop Mode*/
#define EM3 3

#define CURRENT_ENERGY_MODE EM2

/*Time Period = 2250 ms*/
#define LETIMER_PERIOD_MS 3000//2250
/*LED ON Time = 175ms = (2250-175) = 2075*/
#define LETIMER_REQ_PERIOD_MS 2075

#if (CURRENT_ENERGY_MODE == EM3)
 #define ACTUAL_CLK_FREQ  1000    // Frequency of OSC = 1kHz & prescaler = 1
#else
  #define ACTUAL_CLK_FREQ  8192  // Frequency of OSC = 32.768kHz & prescaler = 4
#endif

/*Source: IoT Lectures*/
#define LETIMER0_PERIOD ((LETIMER_PERIOD_MS * ACTUAL_CLK_FREQ) / 1000)

#define LETIMER0_INTERRUPT_DURATION ((LETIMER_REQ_PERIOD_MS * ACTUAL_CLK_FREQ) / 1000)


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

#endif // APP_H

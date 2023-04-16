/*
 * oscillator.h
 *
 *  Created on: Jan 28, 2022
 *  Author: Dhiraj Bennadi
 */

#ifndef SRC_OSCILLATOR_H_
#define SRC_OSCILLATOR_H_

#include "em_cmu.h"
#include "app.h"

/*
 * This function sets the oscillator source
 *
 * Parameters: None
 *
 * Returns: None.
 */
void enableOscillator(void);

/*
 * This function sets the clock for the LFA clock tree
 *
 * Parameters: None
 *
 * Returns: None.
 */
void selectClock(void);

/*
 * This function sets the prescaler
 *
 * Parameters: None
 *
 * Returns: None.
 */
void setClockPrescaler(void);

/*
 * This function enables the LETIMER0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void enableClock(void);

#endif /* SRC_OSCILLATOR_H_ */

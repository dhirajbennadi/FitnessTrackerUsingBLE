/*
 * timers.h
 *
 *  Created on: April 28th 2022
 *  Author: Dhiraj Bennadi
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"

extern uint32_t timeElapsed;
extern volatile uint32_t msTicks;

void initializeParameters(void);
void initLETIMER(void);
void enableLETIMER(void);

void timerWaitUs_irq(uint32_t us_wait);
void timerWaitUs_polled(uint32_t us_wait);

uint32_t letimerMilliseconds(void);
uint32_t getSysTicks(void);

#endif /* SRC_TIMERS_H_ */

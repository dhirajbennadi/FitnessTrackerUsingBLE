/*
 * timers.c
 *
 *  Created on: April 28th 2022
 *  Author: Dhiraj Bennadi
 */
#include "timers.h"
#include "app.h"

/*LoggerGetTimeStamp Count using LETIMER*/
uint32_t timeElapsed = 0;
/*LoggerGetTimeStamp Count using Systick*/
volatile uint32_t msTicks = 0;

#define MAX_US_DELAY         (LETIMER0_PERIOD * 1000)
#define USEC_PER_SEC         (1000000)
#define MAX_VAL_32_BIT       (0xFFFF)

/*Object for Initialization of LETIMER0*/
LETIMER_Init_TypeDef stInitLETIMER;

/*
 * This function initializes the parameters of the
 * LETIMER init structure
 *
 * Parameters: None
 *
 * Returns: None.
 */
void initializeParameters(void)
{
  stInitLETIMER.enable = false;
  stInitLETIMER.debugRun = true;
  stInitLETIMER.comp0Top = true;
  stInitLETIMER.bufTop = false;
  stInitLETIMER.topValue = 0 ; //LETIMER0_PERIOD;
}

/*
 * This function initializes the LETimer0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void initLETIMER(void)
{
  /*Initialize the parameters of the init structure*/
  initializeParameters();

  LETIMER_Init(LETIMER0, &stInitLETIMER);

  /* Update the COMP0 Value */
  LETIMER_CompareSet(LETIMER0, 0, LETIMER0_PERIOD);

  /* Update the COMP1 Value*/
  //LETIMER_CompareSet(LETIMER0, 1, LETIMER0_INTERRUPT_DURATION);

  /* Update the Period*/
  LETIMER_CounterSet(LETIMER0, LETIMER0_PERIOD);

  /* Enable the Interrupts : Underflow Interrupt and COMP1 Interrupt*/
  //LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_UF));

  LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
  LETIMER_IntClear(LETIMER0, LETIMER_IEN_UF);

  /* Enable the Interrupts : Underflow Interrupt and COMP1 Interrupt*/
  //LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_UF | LETIMER_IEN_COMP1));
  LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_UF));
  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  NVIC_EnableIRQ(LETIMER0_IRQn);

  enableLETIMER();

}

/*
 * This function enables the LETimer0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void enableLETIMER(void)
{
  LETIMER_Enable(LETIMER0, true);
}


/*
 * This function waits for the specified by the parameter in us
 *
 * Parameters: us_wait - Wait period
 *
 * Returns: None.
 */
void timerWaitUs_polled(uint32_t us_wait)
{
  uint32_t currentTimerCount = 0;
  uint32_t calibratedValue = 0;

  /*Range Checking*/
  if(us_wait > MAX_US_DELAY)
  {
      us_wait = MAX_US_DELAY;
  }
  currentTimerCount = LETIMER_CounterGet(LETIMER0);
  uint32_t numberOfTicksNeeded = 0;

  numberOfTicksNeeded = (us_wait * LETIMER0_PERIOD);
  numberOfTicksNeeded /=  LETIMER_PERIOD_MS;
  numberOfTicksNeeded *= 1000;
  numberOfTicksNeeded /= 1000000;

  if(currentTimerCount > numberOfTicksNeeded)
    {
      while(LETIMER_CounterGet(LETIMER0) > currentTimerCount - numberOfTicksNeeded)
        {
          ;
        }
    }
  else
    {
      calibratedValue = currentTimerCount - numberOfTicksNeeded;
      calibratedValue = 0xFFFF - calibratedValue;
      calibratedValue = LETIMER0_PERIOD - calibratedValue;

      while(LETIMER_CounterGet(LETIMER0) > calibratedValue)
        {
          ;
        }
    }
}


/*
 * This function waits for the specified by the parameter in us
 *
 * Parameters: us_wait - Wait period
 *
 * Returns: None.
 */
void timerWaitUs_irq(uint32_t us_wait)
{
  uint16_t currentTimerCount = 0;
  uint16_t calibratedValue = 0;
  uint16_t numberOfTicksNeeded = 0;

  /*Range Checking*/
  if(us_wait > MAX_US_DELAY)
  {
      us_wait = MAX_US_DELAY;
  }
  if(us_wait == 0)
  {
    return;
  }

  currentTimerCount = LETIMER_CounterGet(LETIMER0);
  numberOfTicksNeeded = (us_wait * ACTUAL_CLK_FREQ) / USEC_PER_SEC;

  if(currentTimerCount > numberOfTicksNeeded)
  {
    LETIMER_CompareSet(LETIMER0, 1, currentTimerCount - numberOfTicksNeeded);
    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);

  }
  else
  {
    calibratedValue = currentTimerCount - numberOfTicksNeeded;
    calibratedValue = MAX_VAL_32_BIT - calibratedValue;
    calibratedValue = LETIMER0_PERIOD - calibratedValue;

    LETIMER_CompareSet(LETIMER0, 1, calibratedValue);
    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);
  }

}


/*
 * This function returns the time elapsed since the start of the system
 * The reference for this count is LETIMER
 *
 * Parameters: None
 *
 * Returns: uint32_t
 */
uint32_t letimerMilliseconds(void)
{
  return timeElapsed;
}

/*
 * This function returns the time elapsed since the start of the system
 * The reference for this count is Systick Timer
 *
 * Parameters: None
 *
 * Returns: uint32_t
 */
uint32_t getSysTicks(void)
{
  return msTicks;
}



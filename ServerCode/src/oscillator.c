/*
 * oscillator.c
 *
 *  Created on: Jan 28, 2022
 *  Author: Dhiraj Bennadi
 */

#include "oscillator.h"


/*
 * This function sets the oscillator source
 *
 * Parameters: None
 *
 * Returns: None.
 */
void enableOscillator(void)
{
  if(CURRENT_ENERGY_MODE == EM3)
  {
    CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
  }
  else
  {
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  }
}

/*
 * This function sets the clock for the LFA clock tree
 *
 * Parameters: None
 *
 * Returns: None.
 */
void selectClock(void)
{
  if(CURRENT_ENERGY_MODE == EM3)
  {
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
  }
  else
  {
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  }

}


/*
 * This function sets the prescaler
 *
 * Parameters: None
 *
 * Returns: None.
 */
void setClockPrescaler(void)
{
  if(CURRENT_ENERGY_MODE == EM3)
  {
    CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_1);
  }
  else
  {
    CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);
  }
}


/*
 * This function enables the LETIMER0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void enableClock(void)
{
  CMU_ClockEnable(cmuClock_LETIMER0, true);
}

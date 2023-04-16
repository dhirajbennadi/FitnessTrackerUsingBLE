/*
 * irq.c
 *
 *  Created on: Jan 28, 2022
 *  Author: Dhiraj Bennadi
 */

#include "irq.h"
#include "gpio.h"
#include "scheduler.h"
#include "em_i2c.h"
#include "timers.h"
#include "em_gpio.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
/*
 * Interrupt Handler for LETIMER0
 *
 * Parameters: None
 *
 * Returns: None.
 */
void LETIMER0_IRQHandler (void)
{
  uint32_t interruptFlag = 0;

  interruptFlag = LETIMER_IntGetEnabled(LETIMER0);

  LETIMER_IntClear(LETIMER0, interruptFlag);

  if(interruptFlag & LETIMER_IEN_UF)
  {
      schedulerSetEvent_LETIMER();
      timeElapsed += LETIMER_PERIOD_MS;
#ifdef LOGGING_ENABLE
      LOG_INFO("Underflow Interrupt\n\r");
#endif
  }

  if(interruptFlag & LETIMER_IEN_COMP1)
  {
      schedulerSetEvent_LETIMERCompare1();
#ifdef LOGGING_ENABLE
      LOG_INFO("COMP1 Interrupt\n\r");
#endif
  }

} // LETIMER0_IRQHandler()


void I2C0_IRQHandler(void)
{
  // this can be locally defined
  I2C_TransferReturn_TypeDef transferStatus;
  // This shepherds the IC2 transfer along,
  // it’s a state machine! see em_i2c.c
  // It accesses global variables :
  // transferSequence
  // cmd_data
  // read_data
  // that we put into the data structure passed
  // to I2C_TransferInit()
  transferStatus = I2C_Transfer(I2C0);

  if (transferStatus == i2cTransferDone)
  {
    schedulerSetEvent_I2CComplete();
#ifdef LOGGING_ENABLE
    LOG_INFO("I2C Interrupt\n\r");
#endif
  }
  else if(transferStatus == i2cTransferNack)
  {
      schedulerSetEvent_I2CNACK();
  }

} // I2C0_IRQHandler()


/*
 * Interrupt Handler for SysTick Timer
 *
 * Parameters: None
 *
 * Returns: None.
 */
void SysTick_Handler(void)
{
  msTicks++;
}

void GPIO_EVEN_IRQHandler()
{
  uint32_t interruptFlag = 0;

  interruptFlag = GPIO_IntGetEnabled();

#ifdef LOGGING_ENABLE
    LOG_INFO("GPIO Interrupt Flag = %d\n\r", interruptFlag);
#endif

    GPIO_IntClear(interruptFlag);



    if(interruptFlag == (1 << 6))
    {
        schedulerSetEvent_GPIO();
    }
}

void GPIO_ODD_IRQHandler()
{
  uint32_t interruptFlag = 0;

  interruptFlag = GPIO_IntGetEnabled();

  GPIO_IntClear(interruptFlag);

  if(interruptFlag == (1 << 7))
    {
  schedulerSetEvent_GPIO1();
    }
}

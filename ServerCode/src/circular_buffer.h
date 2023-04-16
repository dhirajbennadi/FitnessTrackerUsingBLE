/***********************************************************************
 * @file      circular_buffer.h
 * @version   0.1
 * @brief     Function header/interface file.
 *
 * @author    Dhiraj Bennadi, dhiraj.bennadi@colorado.edu
 * @date      Dec 22, 2021
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Assignment 1.5 - Circular Buffer
 * @due        
 *
 * @resources   
 * 
 */

// Define everything that a caller needs to know



#ifndef __MY_CIRCULAR_BUFFER__
#define __MY_CIRCULAR_BUFFER__


#include <stdint.h>
#include <stdbool.h>

// This is the number of entries in the queue. Please leave
// this value set to 16.
#define QUEUE_DEPTH      (32)
// Student edit: 
//   define this to 1 if your design uses all array entries
//   define this to 0 if your design leaves 1 array entry empty
#define USE_ALL_ENTRIES  (1)


typedef struct{
  uint16_t charHandle;
  uint32_t bufferLength;
  uint8_t buffer[2];
}circularBuffer_t;

typedef struct{
  uint32_t wptr;
  uint32_t rptr;
  bool isCBFull;
  bool isCBEmpty;
}circularBuffer_Stats;

// function prototypes
bool write_queue (circularBuffer_t buttonIndicationSt);
bool read_queue (circularBuffer_t *ptr);
void get_queue_status (circularBuffer_Stats *buttonIndicationNodeStats);
uint32_t get_queue_depth (void);



#endif // __MY_CIRCULAR_BUFFER__


/***********************************************************************
 * @file      circular_buffer.c
 * @version   0.1
 * @brief     Function implementation file.
 *
 * @author    Dhiraj Bennadi, dhiraj.bennadi@colorado.edu
 * @date      Jan 21, 2022
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#include "circular_buffer.h"


// Declare memory for the queue/buffer/fifo, and the write and read pointers
circularBuffer_t   my_queue[QUEUE_DEPTH]; // the queue
uint32_t         wptr = 0;              // write pointer
uint32_t         rptr = 0;              // read pointer
uint32_t         length = 0;              // read pointer
// Decide how will you handle the full condition. 



// ---------------------------------------------------------------------
// Private function used only by this .c file. 
// compute next ptr value
// Isolation of functionality: This defines "how" a pointer advances. 
// ---------------------------------------------------------------------
static uint32_t nextPtr(uint32_t ptr) {

  // Student edit:
  // Create this function
  uint32_t retVal;
  
  if(ptr < QUEUE_DEPTH)
  {
    if(ptr + 1 == QUEUE_DEPTH)
    {
      retVal =  0;
    }
    else
    {
      retVal = ptr + 1;
    }
  }
    
  return retVal;

} // nextPtr()


// ---------------------------------------------------------------------
// Public function
// This function writes an entry to the queue.
// Returns false if successful or true if writing to a full fifo.
// ---------------------------------------------------------------------
bool write_queue (circularBuffer_t buttonIndicationSt) {

  // Student edit:
  // Create this function
  // Decide how you want to handle the "full" condition.
  if(get_queue_depth() == QUEUE_DEPTH)
  {
    return true;
  }
  else
  {
    my_queue[wptr] = buttonIndicationSt;
    wptr = nextPtr(wptr);
    length++;
    return false;
  }
  // Isolation of functionality: 
  //     Create the logic for "when" a pointer advances



} // write_queue() 


// ---------------------------------------------------------------------
// Public function
// This function reads an entry from the queue.
// Returns false if successful or true if reading from an empty fifo. 
// ---------------------------------------------------------------------
bool read_queue (circularBuffer_t *ptr) {

  // Student edit:
  // Create this function
  if(get_queue_depth() == 0)
  {
    return true;
  }
  else
  {
    *ptr = my_queue[rptr];
    rptr = nextPtr(rptr);
    length--;
    return false;
  }
  
  // Isolation of functionality: 
  //     Create the logic for "when" a pointer advances
  
  
  
} // read_queue() 



// ---------------------------------------------------------------------
// Public function
// This function returns the wptr, rptr, full and empty values.
// ---------------------------------------------------------------------
void get_queue_status (circularBuffer_Stats *buttonIndicationNodeStats){

  // Student edit:
  // Create this function
  buttonIndicationNodeStats->wptr = wptr;
  buttonIndicationNodeStats->rptr = rptr;

  if(get_queue_depth() == QUEUE_DEPTH)
  {
      buttonIndicationNodeStats->isCBFull = true;
  }
  else
  {
      buttonIndicationNodeStats->isCBFull = false;
  }

  if(get_queue_depth() == 0)
  {
      buttonIndicationNodeStats->isCBEmpty = true;
  }
  else
  {
      buttonIndicationNodeStats->isCBEmpty = false;
  }
  


} // get_queue_status() 



// ---------------------------------------------------------------------
// Public function
// Function that computes the number of entries in the queue
// ---------------------------------------------------------------------
uint32_t get_queue_depth() {

  // Student edit:
  // Create this function
  //return (wptr - rptr) % QUEUE_DEPTH;
  return length;
  

} // get_queue_depth() 









/* @author Dario Albani
 * Functions to generate and manage a buffer of message_t
 * that can be used to simulate several communications protocols
 */

#include "kilolib.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

typedef struct msg_node {
  struct msg_node* next;    // the next node in the list
  message_t msg;    // the message structure as for the kb
  uint32_t time_stamp;         // the time stamp or eventually ID of the message
  char been_rebroadcasted;  // if the message has been rebroadcasted
} node_t;

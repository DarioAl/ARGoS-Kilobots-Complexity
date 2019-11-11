/*
 * Kilobot control software for a decision making simulation over different resources.
 * The code is intended to use ALF and to be crosscompiled for Kilobots and ARK.
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#include "kilolib.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "distribution_functions.c"


#define RESOURCES_SIZE 1

#define USE_BUFFER
#ifdef USE_BUFFER
#define BUFFER_SIZE 10
#else
#define BUFFER_SIZE 1
#endif


/*-------------------------------------------------------------------*/
/* General Variables                                                 */
/*-------------------------------------------------------------------*/

/* enum for boolean flags */
typedef enum {
              false = 0,
              true = 1,
} bool;


/*-------------------------------------------------------------------*/
/* Motion Variables                                                 */
/*-------------------------------------------------------------------*/

/* enum for different motion types */
typedef enum {
              FORWARD = 0,
              TURN_LEFT = 1,
              TURN_RIGHT = 2,
              STOP = 3,
} motion_t;

/* current motion type */
motion_t current_motion_type = FORWARD;

/* counters for motion, turning and random_walk */
const double std_motion_steps = 5*16;
const double levy_exponent = 2; // 2 is brownian like motion
const double  crw_exponent = 0.1; // higher more straight
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states/position w.r.t. the smart arena */
typedef enum {
              OUTSIDE_AREA=255,
              INSIDE_AREA_0=0,
              INSIDE_AREA_1=1,
              INSIDE_AREA_2=2,
              INSIDE_AREA_3=3,
              INSIDE_AREA_4=4,
              INSIDE_AREA_5=5,
              INSIDE_AREA_6=6,
              INSIDE_AREA_7=7,
              INSIDE_AREA_8=8,
              INSIDE_AREA_9=9,
} arena_t;

/* enum for keeping trace of internal kilobots decision related states */
typedef enum {
              NOT_COMMITTED=255,
              COMMITTED_AREA_0=0,
              COMMITTED_AREA_1=1,
              COMMITTED_AREA_2=2,
              COMMITTED_AREA_3=4,
              COMMITTED_AREA_4=5,
              COMMITTED_AREA_5=5,
              COMMITTED_AREA_6=6,
              COMMITTED_AREA_7=7,
              COMMITTED_AREA_8=8,
              COMMITTED_AREA_9=9,
} decision_t;

/* current state */
arena_t current_arena_state = OUTSIDE_AREA;
decision_t current_decision_state = NOT_COMMITTED;
bool internal_error = false;

/* EMA alpha */
const double stocazzo = 0.5;
/* Variables for Smart Arena messages */
uint8_t sa_type = 0; // smart arena type (i.e. resource id)
uint8_t resources_hits[RESOURCES_SIZE]; // number of hits for each resource, to be compute by mean of an exp avg
uint8_t resources_pops[RESOURCES_SIZE]; // keep local knowledge about resources
uint8_t resources_umin[RESOURCES_SIZE]; // keep local knowledge about resources umin

/*-------------------------------------------------------------------*/
/* Decision Making                                                   */
/*-------------------------------------------------------------------*/
uint32_t last_decision_ticks = 0;
/* processes variables */
const double k = 0.4;
const double h = 0.4;
/* explore for a bit, estimate the pop and then take a decision */
uint32_t last_decision_tick = 0; /* when last decision was taken */
uint32_t exploration_ticks = 25; /* take a decision only after exploring the environment */

/*-------------------------------------------------------------------*/
/* Communication                                                     */
/*-------------------------------------------------------------------*/
/* flag for message sent */
uint8_t sent_message = 0;

/* current kb message out */
message_t interactive_message;

/* messages are valid for valid_util ticks */
uint8_t valid_until = 100;

/* buffer definitions storing information from other kbs */
uint8_t buffer_iterator = 255;
typedef struct compressed_messsage {
  uint32_t time_received;   // time stamp
  uint8_t current_decision; // agent decision
  uint8_t resource_id;      // id of the current resource
  uint8_t resource_pop;     // estimated population of the current resource
} compressed_messsage;
compressed_messsage last_received_messages[BUFFER_SIZE];
uint8_t messages_count;

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
  if(current_motion_type != new_motion_type ) {
    switch( new_motion_type ) {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left,kilo_straight_right);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left,0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0,kilo_turn_right);
      break;
    case STOP:
    default:
      set_motors(0,0);
    }
    current_motion_type = new_motion_type;
  }
}


/*-------------------------------------------------------------------*/
/* Merge received information about the population of the area on    */
/* which the kb is on. Kilobots can only perceive locally and try to */
/* estimate the population of a resource                             */
/*-------------------------------------------------------------------*/
void exponential_average(uint8_t resource_id, uint8_t resource_pop) {
  // update by using exponential moving averagae to update estimated population
  /* https://stackoverflow.com/questions/37300684/implementing-exponential-moving-average-in-c */
  resources_pops[resource_id] = (uint8_t)(resources_pops[resource_id])*stocazzo+(1-stocazzo)*resource_pop;
}

void merge_scan(bool time_window_is_over) {
  if(time_window_is_over && messages_count>0) {
    for(uint8_t i=0; i<RESOURCES_SIZE; i++) {
      // counts all other resoruces hits
      uint8_t hits_otherResources = 0;
      for(uint8_t j=0; j<RESOURCES_SIZE; j++) {
        if(i!=j)
          hits_otherResources += resources_hits[j];
      }
      /* hits_i/(hits_i+hits_empty) * (1-(hits_otherResources/all_hits))   */
      uint8_t estimated_pop = 255;
      if(hits_otherResources+resources_hits[i] != messages_count) {
        // avoid 0 division
        estimated_pop = 255*(1-(resources_hits[i]/(messages_count-hits_otherResources+resources_hits[i]) * (1-(hits_otherResources/messages_count))));
      }
      // update by mean of exponential moving average
      exponential_average(i, estimated_pop);
    }
    // reset hits counts
    memset(resources_hits, 0, sizeof(resources_hits));
    // reset message count
    messages_count = 0;
  } else {
    // simply add the hit to the hits count and keep count of messages
    if(current_arena_state != 255) {
      resources_hits[current_arena_state]++;
      messages_count++;
    }
  }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the complexity_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for ark / type 1 for kbs interactive msgs                  */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/

void message_rx(message_t *msg, distance_measurement_t *d) {
  /* get id (always firt byte) */
  uint8_t id = msg->data[0];

  if(msg->type==0 && id==kilo_uid) {
    /* ----------------------------------*/
    /* smart arena message               */
    /* ----------------------------------*/

    // the smart arena type is where the kb is first byte of the payload
    // can be NONE (255) or resource id 0<id<254
    current_arena_state = (msg->data[1]); // get resource position
    // update only if not committed or committed and not in area
    // NOTE this works until the indexes for the two enums are ordered!!!
    if(current_decision_state == 255 ||
       (current_decision_state != 255 && current_decision_state != current_arena_state)) {
      resources_umin[current_arena_state] = (msg->data[2]); // get umin for the resource
      merge_scan(false);
    }

    // UNCOMMENT IF NEEDED
    // get arena coordinates
    /* my_coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]); */
    /* my_coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]); */
  } else if(msg->type==1 && id!=kilo_uid && msg->crc==message_crc(msg)) {
    /* ----------------------------------*/
    /* KB interactive message            */
    /* ----------------------------------*/

    // parse the message
    compressed_messsage c_message;
    c_message.time_received = kilo_ticks;
    c_message.current_decision = msg->data[1];
    c_message.resource_id = msg->data[2];
    // only store this one if the agent is committed
    // we are not going to use it anyway but it is bettere to avoid bad data
    if (msg->data[1] != 255) {
      c_message.resource_pop = msg->data[3];
    }

    // store the message in the buffer
    if(buffer_iterator == 255) {
      for(uint8_t i=0; i<BUFFER_SIZE; i++) {
        // trick to avoid accessing bad data at the very beginning
        // this will be filled very soon with good data anyway
        last_received_messages[1] = c_message;
      }
      buffer_iterator = 0; // check if this is the first message we receive
    } else {
      last_received_messages[buffer_iterator] = c_message;

      // circular buffer
      if(buffer_iterator == BUFFER_SIZE-1) {
        buffer_iterator = 0;
      } else {
        buffer_iterator++;
      }
    }

    // now check for received utilities
    // and use these to update our perceived utilities
    exponential_average(msg->data[3], msg->data[4]);
    if(RESOURCES_SIZE>1) {
      exponential_average(msg->data[5], msg->data[6]);
    }

    // UNCOMMENT IF NEEDED
    // get other kb coordinates
    /* the_local_knowledge[id].coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]); */
    /* the_local_knowledge[id].coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]); */
  }
}

/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
  /* this one is filled in the loop */
  return &interactive_message;
}

/*-------------------------------------------------------------------*/
/* successful transmission callback                                  */
/*-------------------------------------------------------------------*/
void message_tx_success() {
  sent_message = 1;
}

/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision() {
  /* Start decision process */
  if(current_decision_state == NOT_COMMITTED) {
    uint8_t processes[RESOURCES_SIZE+1] = {0}; // store here all committment processes

    /****************************************************/
    /* spontaneous commitment process through discovery */
    /****************************************************/
    uint16_t sum_committments = 0;

    for(int i=0; i<RESOURCES_SIZE; i++) {
      if(resources_pops[i] > resources_umin[i]) {
        // normalize between 0 and 255 according to k
        processes[i] = resources_pops[i]*h;
        sum_committments += processes[i];
      }
    }

    /****************************************************/
    /* recruitment over a random agent                  */
    /****************************************************/
    compressed_messsage recruitment_message = last_received_messages[0];
    if(buffer_iterator != 255) {
      uint8_t index = rand_hard()*BUFFER_SIZE;
      recruitment_message = last_received_messages[index];
      // if the message is valid and
      // the agent sending it committed and
      // the population is above umin

      if(kilo_ticks-recruitment_message.time_received < valid_until &&
         recruitment_message.current_decision != 255 &&
         recruitment_message.resource_pop > resources_umin[recruitment_message.resource_id]) {
        processes[RESOURCES_SIZE] = recruitment_message.resource_pop*k;
      }
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(sum_committments+processes[RESOURCES_SIZE] > 255) {
      internal_error = true;
      return;
    }

    uint8_t extraction = rand_hard(); // a random number to extract next decision
    for(int i=0; i<RESOURCES_SIZE; i++) {
      extraction -= processes[i];
      if(extraction <= 0) {
        current_decision_state = i;
      }
    }
    extraction -= processes[RESOURCES_SIZE];
    if(extraction <= 0) {
      current_decision_state = recruitment_message.resource_id;
    }
  } else {

    /****************************************************/
    /* abandon                                          */
    /****************************************************/
    uint8_t abandon = 0;
    /* leave immediately if reached the threshold */
    if(resources_pops[current_decision_state] <= resources_umin[current_decision_state]) {
      abandon = 255*k;
    }

    /****************************************************/
    /* cross inhibtion over a random agent              */
    /****************************************************/

    uint8_t cross_inhibition = 0;
    compressed_messsage cross_message;
    if(buffer_iterator != 255) { // FIXME this only check for the first message
      uint8_t index = rand_hard()*BUFFER_SIZE;
      cross_message = last_received_messages[index];
      // if the message is valid and
      // the agent sending it committed and
      // the population is above umin
      // TODO population
      if(kilo_ticks-cross_message.time_received < valid_until &&
         cross_message.current_decision != 255 &&
         cross_message.resource_pop > resources_umin[cross_message.resource_id]) {
        cross_inhibition = cross_message.resource_pop*k;
      }
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(abandon+cross_inhibition > 255) {
      internal_error = true;
      return;
    }

    uint8_t extraction = rand_hard(); // a random number to extract next decision
    extraction -= (abandon+cross_inhibition);
    if(extraction <= 0) {
      current_decision_state = NOT_COMMITTED;
      return;
    }
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
  switch (current_motion_type) {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if (kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;
  case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks) {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      if (rand_hard() % 2) {
        set_motion(TURN_LEFT);
      } else {
        set_motion(TURN_RIGHT);
      }
      // compute turning time
      double angle = 0;
      if(crw_exponent == 0) {
          angle = (uniform_distribution(0, (M_PI)));
      } else {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
    break;

  case STOP:
  default:
    set_motion(FORWARD);
  }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup() {
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  /* Initialise LED and motors */
  set_color(RGB(3,3,3));
  /* Initialise motion variables */
  set_motion(FORWARD);
  for(uint8_t i; i<RESOURCES_SIZE; i++) {
    resources_hits[i] = 0;
    resources_pops[i] = 125;
    resources_umin[i] = 0;
  }
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
  // visual debug. Signal internal decision errors
  while(internal_error) {
    set_color(RGB(0,3,0));
    delay(500);
    set_color(RGB(0,0,3));
    delay(500);
  }

  /* take decision only after exploration */
  if(exploration_ticks <= kilo_ticks-last_decision_ticks) {
    merge_scan(true);
    // UNCOMMENT after
    /* take_decision(); */
    /*--------------------*/
    // reset last decision ticks
    last_decision_ticks = kilo_ticks;
    }

  // REMOVE after
  current_decision_state == COMMITTED_AREA_0;
  if(current_arena_state == INSIDE_AREA_0) {
    set_color(RGB(0,3,0));
    set_motion(STOP);
  } else {
    random_walk();
  }
  // UNCOMMENT after
  /* if(current_decision_state != NOT_COMMITTED) { */
  /*   // if over the wanted resource */
  /*   if(current_decision_state == current_arena_state) { */
  /*     // turn or green led if status is committed and over the area */
  /*     set_color(RGB(0,3,0)); */
  /*     // stop and exploit the area */
  /*     set_motion(STOP); */
  /*   } else { */
  /*     // turn on red led if status is committed but still loking for the area */
  /*     set_color(RGB(3,0,0)); */
  /*     random_walk(); // looking for the wanted resource */
  /*   } */
  /* } else { */
  /*   // simply continue as uncommitted and explore */
  /*   set_color(RGB(3,3,3)); */
  /*   random_walk(); */
  /* } */
  /*--------------------*/

  /* fill up message type. Type 1 used for kbs */
  interactive_message.type = 1;
  /* fill up the current kb id */
  interactive_message.data[0] = kilo_uid;
  /* fill up the current states */
  interactive_message.data[1] = current_decision_state;
  interactive_message.data[2] = current_arena_state;

  /* if committed share your utility */
  uint8_t rand_resource_1, rand_resource_2;
  if(current_decision_state != 255) {
    rand_resource_1 = current_decision_state; // this is set to avoid sharing same resources
    interactive_message.data[3] = resources_pops[current_decision_state];
    interactive_message.data[4] = resources_pops[current_decision_state];
  } else {
    /* share a random utility */
    rand_resource_1 = rand_hard()*RESOURCES_SIZE;
    interactive_message.data[3] = rand_resource_1;
    interactive_message.data[4] = resources_pops[rand_resource_1];
  }
  /* share also a second random other utility */
  if(RESOURCES_SIZE>1) {
    do {
      rand_resource_2 = rand_hard()*RESOURCES_SIZE;
    }
    while(rand_resource_2 == current_decision_state && rand_resource_2 == rand_resource_1);

    interactive_message.data[5] = rand_resource_2;
    interactive_message.data[6] = resources_pops[rand_resource_2];
    resources_pops[0] = 1;
  }
  /* fill up the crc */
  interactive_message.crc = message_crc(&interactive_message);

  // UNCOMMENT IF NEEDED
  /* fill up the message of uint8 by splitting the uint16 */
  /* interactive_message.data[2] = (my_coordinates.x >> 8); // hi part of the uint16 */
  /* interactive_message.data[3] = (my_coordinates.x & 0xff); // lo part of the uint16 */
  /* interactive_message.data[4] = (my_coordinates.y >> 8); // hi part of the uint16 */
  /* interactive_message.data[5] = (my_coordinates.y & 0xff); // lo part of the uint16 */
}

int main() {
  kilo_init();
  // register message reception callback
  kilo_message_rx = message_rx;
  // register message transmission callback
  kilo_message_tx = message_tx;
  // register tranmsission success callback
  kilo_message_tx_success = message_tx_success;

  kilo_start(setup, loop);

  return 0;
}

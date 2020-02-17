/*
 * Kilobot control software for a decision making simulation over different resources.
 *
 * Explanation about the code:
 * The kilobots are expected to exploit a set of resources displaced over the arena.
 * The kilobots are equipped (this is a simulated sensor) witth a sensor that allows them to sense the region of the arena
 * directly beneath.
 * The kilobots can only estimate the overall resources statuses and to do so they use and exponential moving average
 * computed over time and on multiple scans.
 *
 * The kilobots implement a stochastic strategy for exploiting the resources and switch between three different statuses:
 * - uncommitted - exploring and not exploiting any resource (BLUE LED)
 * - committed and looking for the resource - actively looking for a region cotaining the resource (RED LED)
 * - committed and working - working over the are and exploiting the resource. In this phase the kilobot stands still (GREEN LED)
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#include "kilolib.h"
// do not change the order of includes here
// used for debug with ARGoS simulator
#include "complexity.h"
#ifdef DEBUG_KILOBOT
#include <debug.h>
#endif
#include "distribution_functions.c"
#include "message_t_list.c"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

// define the resources to be expected in the current simulation
#define RESOURCES_SIZE 3

/* enum for boolean flags */
typedef enum {
              false = 0,
              true = 1,
} bool;


/*-------------------------------------------------------------------*/
/* Motion Variables                                                  */
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
const double  crw_exponent = 0.0; // higher more straight
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

// the kb is biased toward the center when close to the border
double rotation_to_center = 0; // if not 0 rotate toward the center (use to avoid being stuck)

/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states/position w.r.t. the smart arena */
typedef enum {
              OUTSIDE_AREA=255,
              INSIDE_AREA_0=0,
              INSIDE_AREA_1=1,
              INSIDE_AREA_2=2,
} arena_t;

/* enum for keeping trace of internal kilobots decision related states */
typedef enum {
              NOT_COMMITTED=255,
              COMMITTED_AREA_0=0,
              COMMITTED_AREA_1=1,
              COMMITTED_AREA_2=2,
} decision_t;

/* current state */
arena_t current_arena_state = OUTSIDE_AREA;
decision_t current_decision_state = NOT_COMMITTED;
/* variable to signal internal computation error */
bool internal_error = false;

/* Exponential Moving Average alpha */
const double ema_alpha = 0.75;
/* Variables for Smart Arena messages */
uint8_t resources_hits[RESOURCES_SIZE]; // number of hits for each resource, to be compute by mean of an exp avg
uint8_t resources_pops[RESOURCES_SIZE]; // keep local knowledge about resources
uint8_t resources_umin[RESOURCES_SIZE]; // keep local knowledge about resources umin

/*-------------------------------------------------------------------*/
/* Decision Making                                                   */
/*-------------------------------------------------------------------*/
uint32_t last_decision_ticks = 0;
/* processes variables */
const double k = 0.4; // determines the spontaneous (i.e. based on own information) processes weight
const double h = 0.4; // determines the interactive (i.e. kilobot-kilobot) processes weight
/* explore for a bit, estimate the pop and then take a decision */
uint32_t last_decision_tick = 0; /* when last decision was taken */
uint32_t exploration_ticks = 250; /* take a decision only after exploring the environment */

/*-------------------------------------------------------------------*/
/* Communication                                                     */
/*-------------------------------------------------------------------*/
/* flag for message sent */
uint8_t sent_message = 0;

/* current kb message out */
message_t interactive_message;

/* messages are valid for valid_util ticks */
uint8_t valid_until = 100;

/* buffer for communications */
/* used both for flooding protocol and for dm */
node_t *b_head = NULL;
/* count messages from smart arena */
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
  resources_pops[resource_id] = resource_pop*(ema_alpha) + resources_pops[resource_id]*(1-ema_alpha);

#ifdef DEBUG_KILOBOT
  /**** save DEBUG information ****/
  /* printf("DARIO rp %d - rps %d - ealpha %f \n", resource_pop, resources_pops[resource_id], ema_alpha); */
  /* printf("----------------------------- \n"); */
  /* fflush(stdout); */
  if(resource_id == 0)
    debug_info_set(ema_resource0, resources_pops[resource_id]);
  else if(resource_id == 1)
    debug_info_set(ema_resource1, resources_pops[resource_id]);
  else if(resource_id == 2)
    debug_info_set(ema_resource2, resources_pops[resource_id]);
#endif
}


/* -------------------------------------*/
/* Compute estimation for res given the */
/* other two resources ores1 and ores2  */
/* Use message count to normalize       */
/* -------------------------------------*/

uint8_t estimate_population(uint8_t res, uint8_t ores1, uint8_t ores2, uint8_t msg_count) {
  /* compute according to the following formula:                     */
  /* hits_i/(hits_i+hits_empty) * (1-(hits_otherResources/all_hits)) */
  // Avoid computing if estimated_pop should be 255
  if(res != msg_count) {
    double no_ores = res+(msg_count-res+ores1+ores2);
    double all_ores = ores1+ores2;
    return  (uint8_t) (255*((double)res/no_ores) * (1-(all_ores/(double)msg_count)));
  } else {
    return 255;
  }
}


/* ------------------------------------- */
/* Merge different scan from the same kb */
/* or compute the population if the time */
/* for estimation has passed             */
/* ------------------------------------- */

void merge_scan(bool time_window_is_over) {
  // iterator for cycles
  uint8_t i;

  // only merge after a time window and if there are messages
  if(time_window_is_over) {
    if(messages_count == 0) {
      // time window for estimation closed but no messages
      // signal internal error
      internal_error = true;
      return;
    }

#ifdef DEBUG_KILOBOT
    /**** save DEBUG information ****/
    debug_info_set(num_messages, messages_count);
    uint8_t allres = 0;
    for(i=0; i<RESOURCES_SIZE; i++) {
      allres += resources_hits[i];
    }
    debug_info_set(hits_empty, messages_count-allres);
    /********************************/
#endif

    for(i=0; i<RESOURCES_SIZE; i++) {
      if(i == 0) {
#ifdef DEBUG_KILOBOT
        /**** save DEBUG information ****/
        debug_info_set(hits_resource0, resources_hits[i]);
        /********************************/
#endif

        // update by mean of exponential moving average
        uint8_t e_pop = estimate_population(resources_hits[0], resources_hits[1], resources_hits[2], messages_count);
        exponential_average(i, e_pop);
      } else if(i == 1) {

#ifdef DEBUG_KILOBOT
        /**** save DEBUG information ****/
        debug_info_set(hits_resource1, resources_hits[i]);
        /********************************/
#endif

        // update by mean of exponential moving average
        uint8_t e_pop = estimate_population(resources_hits[1], resources_hits[0], resources_hits[2], messages_count);
        exponential_average(i, e_pop);
      } else if(i == 2) {

#ifdef DEBUG_KILOBOT
        /**** save DEBUG information ****/
        debug_info_set(hits_resource2, resources_hits[i]);
        /********************************/
#endif

        // update by mean of exponential moving average
        uint8_t e_pop = estimate_population(resources_hits[2], resources_hits[0], resources_hits[1], messages_count);
        exponential_average(i, e_pop);
      }
    }

    // reset hits counts
    memset(resources_hits, 0, sizeof(resources_hits));
    // reset message count
    messages_count = 0;
  } else {
    // add the hit to the hits count
    if(current_arena_state != 255) {
      resources_hits[current_arena_state]++;
    }
  }
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the complexity_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for arena / type 1 for kbs interactive msgs                */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/

/* see next function for specification of what is done here */
void parse_smart_arena_data(uint8_t data[9], uint8_t kb_position) {
  // update message count
  messages_count++;

  // index of first element in the data
  uint8_t shift = kb_position*3;
  // get arena state
  int _arenastate = (data[1+shift] &0x60) >> 5;
  // convert to 0,1,2 for resources and 255 for empty
  if(_arenastate == 0) {
    current_arena_state = 255;
  } else {
    current_arena_state = _arenastate-1;
  }

  // get umin for resource if any
  if(current_arena_state != 255) {
    int _umin = (data[1+shift] &0x1E) >> 1;
    resources_umin[current_arena_state] = _umin;
  }
  // get rotation toward the center (if far from center)
  int _rotationsign = (data[1+shift] &0x01);
  int _rotation = data[2+shift];
  rotation_to_center = ((double)_rotation*M_PI)/180.0;
  if(_rotationsign == 1) {
    rotation_to_center = -1 * rotation_to_center;
  }
}


void message_rx(message_t *msg, distance_measurement_t *d) {
  // if type 0 is either from ARGoS or ARK
  if(msg->type==0) {
    /* ----------------------------------*/
    /* smart arena message               */
    /* ----------------------------------*/

    /* see README.md to understand about ARK messaging */
    /* data has 3x24 bits divided as                   */
    /*  data[0]   data[1]   data[2]                    */
    /* xxxx xxxx xyyz zzzw wwww wwww                   */
    /* x bits used for kilobot id                      */
    /* y bits used for kilobot arena state             */
    /* z bits used for resource umin                   */
    /* w bits used for kilobot rotattion toward center */

    // unpack message
    bool message_received = false;
    // ids are first 9 bits
    int id1 = msg->data[0] << 1 | msg->data[1] >> 7;
    int id2 = msg->data[3] << 1 | msg->data[4] >> 7;
    int id3 = msg->data[6] << 1 | msg->data[7] >> 7;

    if(id1 == kilo_uid) {
      parse_smart_arena_data(msg->data, 0);
      message_received = true;
    } else if(id2 == kilo_uid) {
      parse_smart_arena_data(msg->data, 1);
      message_received = true;
    } else if (id3 == kilo_uid) {
      parse_smart_arena_data(msg->data, 2);
      message_received = true;
    }

    // if received a message
    if(message_received) {
      // if uncommitted or committed and not working on area
      // if the kilobot is working on an area we decide not to update his scans
      if(current_decision_state == 255 || (current_decision_state != 255 && current_decision_state != current_arena_state)) {
          merge_scan(false);
      }
    }

  } else if(msg->type==1) {
    /* get id (always firt byte when coming from another kb) */
    uint8_t id = msg->data[0];

    // check that is a valid crc and another kb
    if(id!=kilo_uid && msg->crc==message_crc(msg)){
      /* ----------------------------------*/
      /* KB interactive message            */
      /* ----------------------------------*/

      // increase messages_count for average estimation
      messages_count++;

      // check received message and merge info
      uint8_t e_pop, ores0, ores1, ores2, msg_count;
      msg_count = msg->data[6];
      ores0 = msg->data[3];
      ores1 = msg->data[4];
      ores2 = msg->data[5];
      uint8_t res_index;
      for(res_index=0; res_index<RESOURCES_SIZE; res_index++) {
        if(res_index == 0) {
          e_pop = estimate_population(ores0, ores1, ores2, msg_count);
        } else if(res_index == 1) {
          e_pop = estimate_population(ores1, ores0, ores2, msg_count);
        } else if(res_index == 2) {
          e_pop = estimate_population(ores2, ores0, ores1, msg_count);
        } else {
          printf("in kb interactive message");
          fflush(stdout);
          internal_error = true;
          return;
        }
        exponential_average(e_pop, res_index);
      }

      // store the message in the buffer for flooding and dm
      // if not stored yet
      if(!b_head) {
        // create the head of the list
        b_head = malloc(sizeof(node_t));
        if(!b_head) {
          internal_error = true;
          return;
        }

        // fill the new one
        b_head->msg = msg;
        b_head->next = NULL;
        b_head->time_stamp = kilo_ticks;
        b_head->been_rebroadcasted = false;
      } else {
        // check if it has been parsed before
        // avoid resending same messages over and over again
        if(msg->data[0] == kilo_uid || mtl_is_message_present(b_head, msg) != -1) {
          // do not store, it is mine
          return;
        }

        // message is new, store it
        mtl_push_back(b_head, msg, kilo_ticks);
      }
    }
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
  // iterator for cycles
  uint8_t i;
  /* Start decision process */
  if(current_decision_state == NOT_COMMITTED) {
    uint8_t processes[RESOURCES_SIZE+1] = {0}; // store here all committment processes

    /****************************************************/
    /* spontaneous commitment process through discovery */
    /****************************************************/
    uint16_t sum_committments = 0;

    for(i=0; i<RESOURCES_SIZE; i++) {
      if(resources_pops[i] > resources_umin[i]) {
        // normalize between 0 and 255 according to k
        processes[i] = resources_pops[i]*h;
        sum_committments += processes[i];
      }
    }

    /****************************************************/
    /* recruitment over a random agent                  */
    /****************************************************/

    // get node from the list
    node_t* recruitment_message = NULL;
    mtl_get_node_at(b_head, rand_soft()*mtl_size(b_head)-1, recruitment_message);

    uint8_t recruiter_state = 255;
    if(recruitment_message) {
      uint8_t recruiter_state = recruitment_message->msg->data[1];
    }

    // if the recruiter is committed and the pop of the area is above umin
    if(recruiter_state != 255 && resources_pops[recruiter_state] > resources_umin[recruiter_state]) {
      // computer recruitment value for current agent
      processes[RESOURCES_SIZE] = resources_pops[recruiter_state]*k;
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(sum_committments+processes[RESOURCES_SIZE] > 255) {
      printf("in sum commitment");
      fflush(stdout);
      internal_error = true;
      return;
    }

    // a random number to extract next decision
    int extraction = rand_soft();
    // subtract commitments
    for(i=0; i<RESOURCES_SIZE; i++) {
      extraction -= processes[i];
      if(extraction <= 0) {
        current_decision_state = i;
        return;
      }
    }
    // subtract recruitment
    extraction -= processes[RESOURCES_SIZE];
    if(extraction <= 0) {
      current_decision_state = recruiter_state;
    }
  } else {

    /****************************************************/
    /* abandon                                          */
    /****************************************************/
    uint8_t abandon = 0;
    /* leave immediately if reached the threshold */
    if(resources_pops[current_decision_state] <= resources_umin[current_decision_state]) {
      abandon = 255*h;
    }

    /****************************************************/
    /* cross inhibtion over a random agent              */
    /****************************************************/

    uint8_t cross_inhibition = 0;
    node_t* cross_message = NULL;
    mtl_get_node_at(b_head, rand_soft()*mtl_size(b_head)-1, cross_message);
    uint8_t inhibitor_state = 255;
    if(cross_message) {
     inhibitor_state = cross_message->msg->data[1];
    }
    // if the recruiter is committed and the pop of the area is above umin
    if(inhibitor_state != 255 && resources_pops[inhibitor_state] > resources_umin[inhibitor_state]) {
      // computer recruitment value for current agent
      cross_inhibition = resources_pops[inhibitor_state]*k;
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(abandon+cross_inhibition > 255) {
      printf("in abandon plus cross");
      fflush(stdout);
      internal_error = true;
      return;
    }

    // a random number to extract next decision
    int extraction = rand_soft();
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
    if(kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

  case FORWARD:
    /* if moved forward for enough time turn */
    if(kilo_ticks > last_motion_ticks + straight_ticks) {
      double angle = 0; // rotation angle

      /* if the smart arena signals a rotation angle then rotate */
      if(rotation_to_center != 0) {
        if(rotation_to_center > 0) {
          set_motion(TURN_RIGHT);
        } else {
          set_motion(TURN_LEFT);
        }
        // when too close to the border bias toward center
        angle = abs(rotation_to_center*2);
      } else {
        /* perform a random turn */
        last_motion_ticks = kilo_ticks;
        if (rand_soft() % 2) {
          set_motion(TURN_LEFT);
        } else {
          set_motion(TURN_RIGHT);
        }
        /* random angle */
        if(crw_exponent == 0) {
          angle = uniform_distribution(0, (M_PI));
        } else {
          angle = fabs(wrapped_cauchy_ppf(crw_exponent));
        }
      }

      /* compute turning time */
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
  uint8_t i;
  for(i=0; i<RESOURCES_SIZE; i++) {
    resources_hits[i] = 0;
    resources_pops[i] = 0;
    resources_umin[i] = 0;
  }
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
  // visual debug. Signal internal decision errors
  // if the kilobots blinks green and blue something was wrong
  while(internal_error) {
    // somewhere over the rainbow....
    set_color(RGB(3,0,0));
    delay(500);
    set_color(RGB(0,3,0));
    delay(500);
    set_color(RGB(0,0,3));
    delay(500);
    set_color(RGB(3,3,0));
    delay(500);
    set_color(RGB(3,0,3));
    delay(500);
    set_color(RGB(0,3,3));
    delay(500);
    set_color(RGB(3,3,3));
    delay(500);
    }

  /*
   * if
   *   it is time to take decision after the exploration the fill up an update message for other kbs
   *   then update utility estimation and take next decision according to the PFSM
   *   NOTE this has higher priority w.r.t. the rebroadcast below. There is no check over the sent_message flag
   * else
   *   continue the exploration by means of random walk and only use the communication medium to
   *   rebroadcast messages
   */
  if(exploration_ticks <= kilo_ticks-last_decision_ticks) {

    // fill my message before resetting the hits
    // fill up message type. Type 1 used for kbs
    interactive_message.type = 1;
    // fill up the current kb id
    interactive_message.data[0] = kilo_uid;
    // fill up the current states
    interactive_message.data[1] = current_decision_state;
    interactive_message.data[2] = current_arena_state;
    // share hits counts for all resources
    uint8_t res_index;
    for(res_index=0; res_index<RESOURCES_SIZE; res_index++) {
      interactive_message.data[3+res_index] = resources_hits[res_index];
    }
    // also send current message count since it is need for averaging
    interactive_message.data[3+RESOURCES_SIZE] = messages_count;
    // fill up the crc
    interactive_message.crc = message_crc(&interactive_message);

    // update utility estimation
    merge_scan(true);

    // it is time to take the next decision
    take_decision();

    // reset last decision ticks
    last_decision_ticks = kilo_ticks;

  } else if(sent_message){
    // reset flag
    sent_message = 0;
    // get first not rebroadcasted message from flooding buffer
    node_t* not_rebroadcasted = NULL;
    mtl_get_not_rebroadcasted(b_head, not_rebroadcasted);

    // check if still valid or old
    while(not_rebroadcasted){
      if(kilo_ticks-not_rebroadcasted->time_stamp > exploration_ticks) {
        // is old, delete
        mtl_remove_node(&b_head, not_rebroadcasted);
        not_rebroadcasted = NULL;
        // get next one
        mtl_get_not_rebroadcasted(b_head, not_rebroadcasted);
      } else {
        // got it, break and set it up for rebroadcast
        not_rebroadcasted->been_rebroadcasted = 1;
        break;
      }
    }

    // if there is a valid message then set it up for rebroadcast
    if(not_rebroadcasted) {
      // set it up for rebroadcast
      interactive_message.type = 1;
      memcpy(interactive_message.data, not_rebroadcasted->msg->data, sizeof(uint8_t));
      interactive_message.crc = not_rebroadcasted->msg->crc;
    }
  }

  /* Now parse the decision and act accordingly */
  if(current_decision_state != NOT_COMMITTED) {
    // if over the wanted resource
    if(current_decision_state == current_arena_state) {
      // turn or green led if status is committed and over the area
      set_color(RGB(0,3,0));
      // stop and exploit the area
      set_motion(STOP);
    } else {
      // turn on red led if status is committed but still loking for the area
      set_color(RGB(3,0,0));
      random_walk(); // looking for the wanted resource
    }
  } else {
    // simply continue as uncommitted and explore
    set_color(RGB(3,3,3));
    random_walk();
  }
}

int main() {
  kilo_init();
  // register message reception callback
  kilo_message_rx = message_rx;
  // register message transmission callback
  kilo_message_tx = message_tx;
  // register tranmsission success callback
  kilo_message_tx_success = message_tx_success;
#ifdef DEBUG_KILOBOT
  // initialize debugging information
  debug_info_create();
#endif
  kilo_start(setup, loop);

  return 0;
}

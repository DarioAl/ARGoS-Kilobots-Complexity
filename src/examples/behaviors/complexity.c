//TODO considera di non usare il flooding in reale oppure fallo in quei due secondi
// usa #ifdef ARGOS_simulator_BUILD
// ogni tanto fa un broadcast che dice ai kbs di iniziare a parlare (delay nel mandare il messaggio su kb)
// ark dice di parlare e nel momento in cui dicono smetti di parlare cambia anche stato e fai decisione
// 18+2
// controlla che il messaggio sia stato mandato tx_succes


/*
 * Kilobot control software for a decision making simulation over different resources.
 *
 * Explanation about the code:
 * The kilobots are expected to exploit a set of resources displaced over the arena.
 * The kilobots are equipped (this is a simulated sensor) with a sensor that allows them to sense the region of the arena
 * directly beneath.
 * The kilobots can only estimate the overall resources statuses and to do so they use and exponential moving average
 * computed over time and on multiple scans.
 *
 * The kilobots implement a stochastic strategy for exploiting the resources and switch between three different statuses:
 * - uncommitted - exploring and not exploiting any resource (BLUE LED)
 * - committed and looking for the resource - actively looking for a region cotaining the resource (RED LED)
 * - committed and working - working over the are and exploiting the resource. In this phase the kilobot stands still (GREEN LED)
 *
 * NOTE increase all ticks by a factor of 10 when dealing with real simulations
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
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.0; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

// the kb is biased toward the center when close to the border
float rotation_to_center = 0; // if not 0 rotate toward the center (use to avoid being stuck)

/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states/position w.r.t. the smart arena */
typedef enum {
              OUTSIDE_AREA=255,
              INSIDE_AREA_0=0,
              INSIDE_AREA_1=1,
              INSIDE_AREA_2=2,
              INSIDE_AREA_01=3,
              INSIDE_AREA_02=6,
              INSIDE_AREA_12=7,
              INSIDE_AREA_012=21,
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
const float ema_alpha = 0.75;
/* Umin threshold for the kb in 255/16 splice */
const uint8_t umin = 0;

/* Variables for Smart Arena messages */
uint8_t temp_resource_pops[RESOURCES_SIZE]; // temporary local knowledge about resources
uint8_t resources_pops[RESOURCES_SIZE]; // keep local knowledge about resources

/*-------------------------------------------------------------------*/
/* Decision Making                                                   */
/*-------------------------------------------------------------------*/
/* system time scale -- a control variable       */
/* used to increase or decrease the system speed */
const float tau = 0.25; // one/fourth of the velocity

/* processes variables */
const float h = 1; // determines the spontaneous (i.e. based on own information) processes weight
const float k = 1; // determines the interactive (i.e. kilobot-kilobot) processes weight

/* explore for a bit, estimate the pop and then take a decision */
uint32_t last_decision_ticks = 0; /* when last decision was taken */
const uint32_t exploration_ticks = 100; /* take a decision only after exploring the environment */

/*-------------------------------------------------------------------*/
/* Communication                                                     */
/*-------------------------------------------------------------------*/
/* flag for message sent */
uint8_t sent_message = 0;

/* turn this flag on if there is a valid message to send */
uint8_t to_send_message;

/* unique msg id */
uint8_t msg_id = 0;

/* current kb message out */
message_t interactive_message;

/* for broacasts */
uint32_t last_broadcast_ticks = 0;
const uint32_t broadcast_ticks = 80;

/* messages are valid for valid_until ticks */
const uint32_t valid_until = 200;

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
      break; case STOP:
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
  resources_pops[resource_id] = (uint8_t)round(((float)resource_pop*(ema_alpha)) + ((float)resources_pops[resource_id]*(1.0-ema_alpha)));

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
#ifdef DEBUG_KILOBOT
      printf("in message count \n");
      fflush(stdout);
#endif
      // time window for estimation closed but no messages
      // signal internal error
      internal_error = true;
      return;
    }

    for(i=0; i<RESOURCES_SIZE; i++) {
      if(temp_resource_pops[i]>0) {
        // update by mean of exponential moving average
        exponential_average(i, temp_resource_pops[i]);
      }
    }

    // reset temp res pops counts
    memset(temp_resource_pops, 0, sizeof(temp_resource_pops));
    // reset message count
    messages_count = 0;
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

  // get arena state by resource (at max 3 resources allowed in the simulation)
  uint8_t ut_a = (data[1+shift] &0x3C) >> 2;
  uint8_t ut_b = (data[1+shift] &0x03) << 2 | (data[2+shift] &0xC0);
  uint8_t ut_c = (data[2+shift] &0x3C) >> 2;
  if(ut_a+ut_b+ut_c == 0) {
    // set this up if over no resource
    current_arena_state = 255;
  } else {
    current_arena_state = 0;
    if(ut_c > 0) {
      // either 0 (on a) or 2 (on c)
      current_arena_state = 2;
    }
    if(ut_b > 0) {
      // either 0 (on a) or 1 (on b) or 7 (on b and c)
      current_arena_state = current_arena_state*3+1;
    }
    if(ut_a > 0) {
      // either 0 (on a) 3 (on a and b) 6 (on a and c) 21 (on a and b and c)
      current_arena_state = current_arena_state*3;
    }
  }

  // store received utility
  // 15 slices of means every 17
  if(ut_a != 0) {
    temp_resource_pops[0] = ut_a*17;
  }
  if(ut_b != 0) {
    temp_resource_pops[1] = ut_b*17;
  }
  if(ut_c != 0) {
    temp_resource_pops[2] = ut_c*17;
  }

  // get rotation toward the center (if far from center)
  // avoid colliding with the wall
  uint8_t rotation_slice = data[2+shift] &0x03;
  if(rotation_slice == 3) {
    rotation_to_center = -M_PI/2;
  } else {
    rotation_to_center = rotation_slice*M_PI/2;
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
    /* xxxx xxxx xxaa aabb bbcc ccyy                   */
    /* x(10) bits used for kilobot id                  */
    /* a(4) bits used for resource a utility           */
    /* b(4) bits used for resource b utility           */
    /* c(4) bits used for resource c utility           */
    /* y(2) bits used for turing angle                 */

    /* How to interpret the data received?             */
    /* If no resource a,b,c utility is received then   */
    /* means that the kb is on empty space             */
    /* On the opposite, if a resource utility is there */
    /* means that the kb is on resource space          */
    /* The turning angle is divided in 4 equal slices  */
    /* pi/4 to 3/4pi - 3/4pi to 5/4pi - 5/4pi to 7/4pi */

    // unpack message
    bool message_received = false;
    // ids are first 10 bits
    int id1 = msg->data[0] << 2 | msg->data[1] >> 6;
    int id2 = msg->data[3] << 2 | msg->data[4] >> 6;
    int id3 = msg->data[6] << 2 | msg->data[7] >> 6;

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
      merge_scan(false);
    }
  } else if(msg->type==1) {
    /* get id (always firt byte when coming from another kb) */
    uint8_t id = msg->data[0];
    // check that is a valid crc and another kb
    if(id!=kilo_uid){
      /* ----------------------------------*/
      /* KB interactive message            */
      /* ----------------------------------*/

      // check received message and merge info and ema
      if(msg->data[3] > 0) {
        exponential_average(0, msg->data[3]);
      }
      if(msg->data[4] > 0) {
        exponential_average(1, msg->data[4]);
      }
      if(msg->data[5] > 0) {
        exponential_average(2, msg->data[5]);
      }
      // store the message in the buffer for flooding and dm
      // if not stored yet
      if(b_head == NULL) {
        // create the head of the list
        b_head = malloc(sizeof(node_t));
        // fill the new one
        b_head->msg = *msg;
        b_head->next = NULL;
        b_head->time_stamp = kilo_ticks;
        b_head->been_rebroadcasted = false;
      } else {
        // check if it has been parsed before
        // avoid resending same messages over and over agai/n
        if(mtl_is_message_present(b_head, *msg) >= 0) {
          // do not store
          return;
        }

        // message is new, store it
        node_t* new_node;
        new_node = malloc(sizeof(node_t));
        new_node->msg = *msg;
        new_node->time_stamp=kilo_ticks;
        new_node->been_rebroadcasted=false;
        mtl_push_back(b_head, new_node);
      }
    }
  }
}


/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/

message_t *message_tx() {
  if(to_send_message) {
    /* this one is filled in the loop */
    to_send_message = false;
    return &interactive_message;
  } else {
    return NULL;
  }
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
      // if over umin threshold
      if(resources_pops[i] > umin) {
        // normalized between 0 and 255
        processes[i] = resources_pops[i]*h*tau;
        sum_committments += processes[i];
      }
    }

    /****************************************************/
    /* recruitment over a random agent                  */
    /****************************************************/
    node_t* recruitment_message = NULL;
    uint8_t recruiter_state = 255;

    // get list size
    unsigned list_size = mtl_size(b_head);
    // if list non empty
    if(list_size > 0) {
      // extract a random node
      uint8_t rand = rand_soft()*(list_size-1);
      // retrieve the node
      recruitment_message = b_head;
      while(recruitment_message && rand>0) {
        // set inhibitor state
        recruiter_state = recruitment_message->msg.data[1];

        recruitment_message = recruitment_message->next;
        rand = rand-1;
      }
    }

    // if the recruiter is committed
    if(recruiter_state != 255) {
      // if over umin threshold
      if(resources_pops[recruiter_state] > umin) {
        // computer recruitment value for current agent
        processes[RESOURCES_SIZE] = (uint8_t)(resources_pops[recruiter_state]*k*tau);
      }
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(sum_committments+processes[RESOURCES_SIZE] > 255) {
#ifdef DEBUG_KILOBOT
      printf("in sum commitment: %d, %d\n", sum_committments, processes[RESOURCES_SIZE]);
      fflush(stdout);
#endif
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
    if(resources_pops[current_decision_state] <= umin) {
      abandon = 255*h*tau;
    }

    /****************************************************/
    /* cross inhibtion over a random agent              */
    /****************************************************/
    uint8_t cross_inhibition = 0;
    node_t *cross_message = NULL;
    uint8_t inhibitor_state = 255;

    // get list size
    unsigned list_size = mtl_size(b_head);
    // if list non empty
    if(list_size > 0) {
      // extract a random node
      uint8_t rand = rand_soft()*(list_size-1);
      // retrieve the node
      cross_message = b_head;
      while(cross_message && rand>0) {
        // set inhibitor state
        inhibitor_state = cross_message->msg.data[1];

        cross_message = cross_message->next;
        rand = rand-1;
      }
    }

    // if the inhibitor is committed
    if(inhibitor_state != 255) {
      // if above umin threshold
      if(resources_pops[inhibitor_state] > umin) {
        // computer recruitment value for current agent
        cross_inhibition = (uint8_t)(resources_pops[inhibitor_state]*k*tau);
      }
    }

    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if(abandon+cross_inhibition > 255) {
#ifdef DEBUG_KILOBOT
      printf("in abandon plus cross \n");
      fflush(stdout);
#endif
      internal_error = true;
      return;
    }

    // a random number to extract next decision
    int extraction = rand_soft();
    // subtract abandon
    extraction -= abandon;
    if(extraction <= 0) {
     current_decision_state = NOT_COMMITTED;
      return;
    }
    // subtract cross-inhibition
    extraction -= cross_inhibition;
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
  /* if the arena signals a rotation, then rotate toward the center immediately */
  if(rotation_to_center != 0) {
    if(rotation_to_center > 0) {
      set_motion(TURN_RIGHT);
    } else {
      set_motion(TURN_LEFT);
    }
    // when too close to the border bias toward center
    float angle = abs(rotation_to_center);

    /* compute turning time */
    turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
    straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    return;
  }

  /* else keep on with normal random walk */
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
      float angle = 0; // rotation angle

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
    temp_resource_pops[i] = 0;
    resources_pops[i] = 0;
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
    // fill my message before resetting the temp resource count
    // fill up message type. Type 1 used for kbs
    interactive_message.type = 1;
    // fill up the current kb id
    interactive_message.data[0] = kilo_uid;
    // fill up the current states
    interactive_message.data[1] = current_decision_state;
    interactive_message.data[2] = current_arena_state;
    // share my resource pop for all resources
    uint8_t res_index;
    for(res_index=0; res_index<RESOURCES_SIZE; res_index++) {
      interactive_message.data[3+res_index] = temp_resource_pops[res_index];
    }

    // last byte used for msg id
    interactive_message.data[8] = msg_id;
    msg_id ++; // update the id after
    // fill up the crc
    interactive_message.crc = message_crc(&interactive_message);

    // tell that we have a msg to send
    to_send_message = true;

    // update utility estimation
    merge_scan(true);

    // it is time to take the next decision
    take_decision();

    // reset last decision ticks
    last_decision_ticks = kilo_ticks;
  } else if(sent_message && broadcast_ticks <= kilo_ticks-last_broadcast_ticks) {
    // get first not rebroadcasted message from flooding buffer
    node_t* not_rebroadcasted = NULL;
    uint16_t nr_pos = mtl_get_first_not_rebroadcasted(b_head, &not_rebroadcasted);

    // check if still valid or old
    while(not_rebroadcasted){
      if(kilo_ticks-not_rebroadcasted->time_stamp > valid_until ) {
        // is old, delete
        not_rebroadcasted = NULL;
        mtl_remove_at(&b_head, nr_pos);
        // get next one
        nr_pos = mtl_get_first_not_rebroadcasted(b_head, &not_rebroadcasted);
      } else {
        // got it, break and set it up for rebroadcast
        not_rebroadcasted->been_rebroadcasted = true;
        break;
      }
    }

    // if there is a valid message then set it up for rebroadcast
    if(not_rebroadcasted) {
      // tell that we have a msg to send
      to_send_message = true;
      // set it up for rebroadcast
      interactive_message.type = 1;
      memcpy(interactive_message.data, not_rebroadcasted->msg.data, sizeof(uint8_t));
      interactive_message.crc = not_rebroadcasted->msg.crc;
    }

    // reset flag
    sent_message = 0;
    last_broadcast_ticks = kilo_ticks;
  }

  current_decision_state = COMMITTED_AREA_0;
  // never stop when exploiting the area
  if(current_arena_state%3 == 0) {
    // turn on red led if status is committed and over the area 0
    set_color(RGB(3,0,0));
  } else {
    // turn on white led if status is committed but still loking for the area
    set_color(RGB(0,0,0));
  }
  random_walk(); // looking for the wanted resource

  /* /\* Now parse the decision and act accordingly *\/ */
  /* if(current_decision_state != NOT_COMMITTED) { */
  /*   // if over empty, turn on white led to signal committed but not working */
  /*   if(current_arena_state == 255) { */
  /*     // if on empty space avoid if below and set led to white */
  /*     // committed but not on the right resource */
  /*     set_color(RGB(3,3,3)); */

  /*   } else { */
  /*     // if over the wanted resource turn on the right led color */
  /*     if(current_decision_state == COMMITTED_AREA_0) { */
  /*       // area 0 */
  /*       if(current_arena_state%3 == 0) { */
  /*         // area 1 is red */
  /*         set_color(RGB(3,0,0)); */
  /*       } else { */
  /*         // turn on white because not on wanted resource */
  /*         set_color(RGB(3,3,3)); */
  /*       } */

  /*     } else if(current_decision_state == COMMITTED_AREA_1) { */
  /*       // area 1 */
  /*       if(current_arena_state == 1 || current_arena_state == 3 || */
  /*          current_arena_state == 7 || current_arena_state == 21) { */
  /*         // area 1 is green */
  /*         set_color(RGB(0,3,0)); */
  /*       } else { */
  /*         // turn on white because not on wanted resource */
  /*         set_color(RGB(3,3,3)); */
  /*       } */

  /*     } else if(current_decision_state == COMMITTED_AREA_2) { */
  /*       // area 2 */
  /*       if(current_arena_state == 2 || current_arena_state == 6 || */
  /*          current_arena_state == 7 || current_arena_state == 21) { */
  /*         // area 2 is blue */
  /*         set_color(RGB(0,0,3)); */
  /*       } else { */
  /*         // turn on white because not on wanted resource */
  /*         set_color(RGB(3,3,3)); */
  /*       } */
  /*     } */
  /*   } */
  /* } else { */
  /*   // simply continue as uncommitted and explore */
  /*   set_color(RGB(0,0,0)); */
  /* } */
  // always random walk, never stop even when exploiting
  /* random_walk(); */
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

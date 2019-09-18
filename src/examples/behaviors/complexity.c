/*
 * Kilobot control software for a decision making simulation over different resources.
 * The code is intended to use ALF and to be crosscompiled for Kilobots and ARK.
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#include "kilolib.h"
#include <stdlib.h>
#include<stdio.h>

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
motion_t current_motion_type = STOP;

/* counters for motion, turning and random_walk */
uint32_t turn_ticks = 60;
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;


/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states */
typedef enum {
              OUTSIDE_AREA=0,
              INSIDE_AREA_A=1,
              INSIDE_AREA_B=2,
} arena_t;

/* current state */
arena_t current_state = OUTSIDE_AREA;

/* pair of coordinates */
typedef struct arena_coordinates arena_coordinates;
struct arena_coordinates {
  // not expressed as in the argos arena
  // x and y are multiplied by 1k
  u_int16_t x; // approximated position
  u_int16_t y; // approximated position
};

/* area information */
typedef  struct arena_area arena_area;
struct arena_area {
  u_int8_t id;
  u_int8_t pop; // normalized between 0-255
};

/* Variables for Smart Arena messages */
u_int8_t sent_message = 0;
u_int8_t sa_type = 0; // 0,1,2 -> outside,in A, in B
arena_area resource_a; // keep local knowledge about resources
arena_area resource_b; // keep local knowledge about resources
arena_coordinates my_coordinates; // current kb coordinates in the arena

/*-------------------------------------------------------------------*/
/* Decision Making                                                   */
/*-------------------------------------------------------------------*/

/* enum for the robot decisions */
typedef enum {
              NONE = 0,
              COMMITTMENT = 1,
              ABANDONDONMENT = 2,
              RECRUITMENT = 3,
              INHIBITION = 4,
} decision_t;

/* local knowledge about a kilobot */
typedef struct local_knowledge local_knowledge;
struct local_knowledge {
  decision_t process; // its last process (used for interactions)
  arena_t state; // kilbots state (in whiche area?)
  arena_coordinates coordinates; // its location
  u_int32_t timestamp; // used for time validation (discard old information)
};

/* the whole local knowledge about other kbs */
/* the index is the id */
local_knowledge the_local_knowledge[100];

/* current decions */
decision_t current_decision = NONE;
/* to share kilobots local knowledge */
message_t interactive_message;


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    bool calibrated = true;
    if ( current_motion_type != new_motion_type ){
        switch( new_motion_type ) {
        case FORWARD:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_straight_left,kilo_straight_right);
            else
                set_motors(70,70);
            break;
        case TURN_LEFT:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_turn_left,0);
            else
                set_motors(70,0);
            break;
        case TURN_RIGHT:
            spinup_motors();
            if (calibrated)
                set_motors(0,kilo_turn_right);
            else
                set_motors(0,70);
            break;
        case STOP:
        default:
            set_motors(0,0);
        }
        current_motion_type = new_motion_type;
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
  // get id (always firt byte)
  u_int8_t id = msg->data[0];

  if(msg->type == 0) {
    /* ----------------------------------*/
    // smart arena message
    /* ----------------------------------*/

    if(id == kilo_uid) {
      // the smart arena type is where the kb is
      // can be NONE, RESOURCE_A and RESOURCE_B
      sa_type = (msg->data[1]); // get arena position

      if(sa_type == INSIDE_AREA_A) {
        resource_a.pop = msg->data[5]; // update area pop
      } else if (sa_type == INSIDE_AREA_B) {
        resource_b.pop = msg->data[5]; // update area pop
      }

      // get arena coordinates
      my_coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]);
      my_coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]);

      if(sa_type == INSIDE_AREA_A) {
        // set color for area A red
        set_color(RGB(3,0,0));
        // stop and exploit
        set_motors(0,0);
        set_motion(STOP);
      } else if(sa_type == INSIDE_AREA_B) {
        // set color for area B green
        set_color(RGB(0,3,0));
        // stop and exploit
        set_motors(0,0);
        set_motion(STOP);
      } else if(sa_type == OUTSIDE_AREA) {
        current_state = OUTSIDE_AREA;
        // search for something
        set_motion(FORWARD);
        last_motion_ticks=kilo_ticks;
      }
    }
  } else if(msg->type == 1) {
    /* ----------------------------------*/
    // KB interactive message
    /* ----------------------------------*/

    if(id != kilo_uid) {
      // check for valid crc
      if(msg->crc == message_crc(msg)) {
        // gather information about other kbs
        // where it is?
        the_local_knowledge[id].state = msg->data[1];
        // get arena coordinates
        the_local_knowledge[id].coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]);
        the_local_knowledge[id].coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]);
        // get process (for interactive processes)
        the_local_knowledge[id].process = msg->data[6];
        // update timestamp
        the_local_knowledge[id].timestamp = kilo_ticks;

        set_color(RGB(3,3,3));

      }
    }
  }
}


/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
  /* this one is filled during the decision process */
  return &interactive_message;
}

/*-------------------------------------------------------------------*/
/* successful transmission callback                                  */
/*-------------------------------------------------------------------*/
void message_tx_success() {
  set_motors(0,0);
  sent_message = 1;
}

/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision() {
  // no valid interactive process yet, set crc to 0
  /* interactive_message.crc = 0; */
  /* interactive_message.type = 1; */

  // get current area utility if any

  // TODO decide using whatever knowledge we have and update the target
  // check status uncommitted or committed

  // if uncommitted
  // spontaneous commitment process through discovery
  // rectruitmet
  // una volta reclutato vai in cerca dell'area A e finche' non la trovi non fai altro

  //if committed
  // spontaneous abandon process
  // cross-inhibition


  /* Fill up the kb message */
  /* interactive_message.data[0] = kilo_uid; */
  /* interactive_message.data[1] = current_state; */
  /* // fill up the message of uint8 by splitting the uin16 */
  /* interactive_message.data[2] = (my_coordinates.x >> 8); // hi part of the uint16 */
  /* interactive_message.data[3] = (my_coordinates.x & 0xff); // lo part of the uint16 */
  /* interactive_message.data[4] = (my_coordinates.y >> 8); // hi part of the uint16 */
  /* interactive_message.data[5] = (my_coordinates.y & 0xff); // lo part of the uint16 */
  /* u_int8_t area_pop = 1; */
  /* interactive_message.data[6] = area_pop; */

  /* // fill up the crc */
  /* interactive_message.crc = message_crc(&interactive_message); */
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
  switch( current_motion_type ) {
  case TURN_LEFT:
    if( kilo_ticks > last_motion_ticks + turning_ticks ) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;  // fixed time FORWARD
      //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
      set_motion(FORWARD);
    }
  case TURN_RIGHT:
    if( kilo_ticks > last_motion_ticks + turning_ticks ) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;  // fixed time FORWARD
      //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
      set_motion(FORWARD);
    }
    break;
  case FORWARD:
    if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
      /* perform a radnom turn */
      last_motion_ticks = kilo_ticks;
      if( rand()%2 ) {
        set_motion(TURN_LEFT);
      }
      else {
        set_motion(TURN_RIGHT);
      }
      turning_ticks = rand()%max_turning_ticks + 1;
    }
    break;
  case STOP:
  default:
    /* stop when reached the area */
    set_motion(STOP);
  }
}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup() {
    /* Initialise LED and motors */
    set_color(RGB(0,0,0));
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    last_motion_ticks=rand()%max_straight_ticks;
    set_motion( FORWARD );
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
  /* set_color(RGB(0,0,0)); */
  /* delay(250); */

  interactive_message.data[0] = 0;
  interactive_message.type = NORMAL;
  interactive_message.crc = message_crc(&interactive_message);

  if(sent_message) {
    sent_message = 0;
    set_color(RGB(0,0,1));
  }

  // take decision
  /* take_decision(); */

  /* if(current_decision == COMMITTMENT || current_decision == RECRUITMENT) { */
  /*   // TODO go to selected area ASAP */
  /*   random_walk(); */
  /* } else if(current_decision == ABANDONDONMENT || current_decision == INHIBITION) { */
  /*   // TODO leave the area ASAP */
  /*   random_walk(); */
  /* } else if(current_decision == NONE) { */
  /*   random_walk(); */
  /* } */
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

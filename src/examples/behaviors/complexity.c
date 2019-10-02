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
#include <limits.h>

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
uint32_t last_decision_ticks = 0;

/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states */
typedef enum {
              OUTSIDE_AREA=0, // over no area
              INSIDE_AREA_A=1, // over (not working on) area a
              INSIDE_AREA_B=2, // over (not working on) area b
              COMMITTED_AREA_A=3, // working on area a
              COMMITTED_AREA_B=4, // working on area b
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
              ABANDON = 2,
              RECRUITMENT_A = 3, // recruitment over resource a
              RECRUITMENT_B = 4, // recruitment over resource b
              INHIBITION_A = 5, // inhibition over resource a
              INHIBITION_B = 6, // inhibition over resource b
} decision_t;

/* area exploitation threshold, when to stop exploitation */
u_int8_t area_exploitation_threshold = 0.5*255;

/* processes variables */
float k = 0.4;
float h = 0.4;

/* local knowledge about a kilobot */
typedef struct local_knowledge local_knowledge;
struct local_knowledge {
  arena_t state; // kilbots state (in whiche area?)
  u_int8_t area_pop; // perceived utility of the area
  arena_coordinates coordinates; // its location
  u_int32_t timestamp; // used for time validation (discard old information)
};
/* the whole local knowledge about other kbs */
/* the index is the id */
u_int8_t num_of_kbs = 25;
local_knowledge the_local_knowledge[25];

/* current decions */
decision_t current_decision = NONE;
/* to share kilobots local knowledge */
message_t interactive_message;

/* explore for a bit, estimate the pop and then take a decision */
u_int32_t last_decision_tick = 0; /* when last decision was taken */
uint32_t exploration_ticks = 250; /* take a decision only after exploring the environment */

/* local knowledge validation period, exploration ticks doubled*/
u_int32_t knowledge_not_valid_after = 250*2; // in ticks


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
/* Merge received information about the population of the area on    */
/* which the kb is on. Kilobots can only perceive locally and try to */
/* estimate the population of a resource                             */
/*-------------------------------------------------------------------*/
void merge_scan() {
  // TODO implementa scansione e stime risorse
  // usa current_state
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
    /* smart arena message               */
    /* ----------------------------------*/

    if(id == kilo_uid) {
      // the smart arena type is where the kb is first byte of the payload
      // can be NONE (255) or resource id 0<id<254
      // TODO change the concept of committed (use a flag if commited) use type to know where on the area we area. If flag committed is on the turno on the led for argos that needs to know where the kb is
      current_state = (msg->data[1]); // get resource position
      merge_scan();

      // UNCOMMENT IF NEEDED
      // get arena coordinates
      /* my_coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]); */
      /* my_coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]); */
    }
  } else if(msg->type == 1) {
    /* ----------------------------------*/
    /* KB interactive message            */
    /* ----------------------------------*/

    if(id != kilo_uid) {
      // check for valid crc
      if(msg->crc == message_crc(msg)) {
        // gather information about other kbs
        // where it is?
        // TODO local knowledge is now a buffer! change that
        the_local_knowledge[id].state = msg->data[1];
        // get arena coordinates
        the_local_knowledge[id].coordinates.x = ((msg->data[2]&0b11) << 8) | (msg->data[3]);
        the_local_knowledge[id].coordinates.y = ((msg->data[4]&0b11) << 8) | (msg->data[5]);
        // get process (for interactive processes)
        the_local_knowledge[id].area_pop = msg->data[6];
        // update timestamp
        the_local_knowledge[id].timestamp = kilo_ticks;
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
  /* no valid interactive process yet, set crc to 0 */
  interactive_message.crc = 0;
  interactive_message.type = 1;

  // get current area utility if any
  u_int8_t area_pop = 0;
  if(current_state == INSIDE_AREA_A) {
    area_pop = resource_a.pop;
  } else if(current_state == INSIDE_AREA_A) {
    area_pop = resource_b.pop;
  }

  /* Start decision process */
  // these are not actually needed but are kept for debug purposes
  u_int8_t commitment = 0;     // value for the commitment process
  u_int8_t abandon = 0;        // value for the abandon process
  u_int8_t recruitment = 0;    // value for the recruitment process
  u_int8_t cross = 0;          // value for the corss-inhibition process


  u_int8_t extraction = rand_soft(); // a random number to extract next decision
  current_decision = NONE; // reset decision process

  // if uncommitted
  if(current_state != COMMITTED_AREA_A &&
     current_state != COMMITTED_AREA_B) {

    /****************************************************/
    /* spontaneous commitment process through discovery */
    /****************************************************/

    // TODO conta numero di timestep passati su un area/risorsa e stima in qualche modo l'utilita' da quelli
    // questo viene fatto nell'intervallo tra una decisione e l'altra

    // fai tutti e due i processi insieme per non prioritizzare il discovery
    // disc disc disc recr

    // if not none then it discovered at least an area (i.e. over an area)
    // if area_pop < 0 then wait for it to recover
    if(current_state != NONE &&
       area_pop > area_exploitation_threshold) {
      commitment = area_pop*k;
    }

    /* is the winning process? */
    extraction -= commitment;
    if(extraction <= 0) {
      extraction = 255; // put it back to 255 to avoid extracion of a second winner
      current_decision = COMMITTMENT; // save the winning process
    }

    /****************************************************/
    /* recruitment over a random agent                  */
    /****************************************************/

    // TODO doppia implementazione:
    // 1) tieni in memoria solo l'ultimo messaggio e usa quello se ancora valido
    // 2) tieni in memoria un buffer di messaggi e randomizza su quelli se ancora validi. uno per robot.
    // questo per vedere come si comporta il sistema se well mixed

    // Continua ad usare l'utilita' stimata dal kb che manda il messaggio

    // da studiare anche come varia questa soluzione in base alla frequenza del take_decision()

    u_int8_t random = (rand_soft()*num_of_kbs)/255; // a random agent
    local_knowledge rand_agent = the_local_knowledge[random];
    // if the message is still valid
    if(rand_agent.timestamp - kilo_ticks < knowledge_not_valid_after) {
      recruitment = rand_agent.area_pop*h;
    }

    /* is the winning process? */
    extraction -= recruitment;
    if(extraction <= 0) {
      extraction = 255; // put it back to 255 to avoid extracion of a second winner
      if(rand_agent.state == COMMITTED_AREA_A) {
        current_decision = RECRUITMENT_A; // save the winning process
      } else {
        current_decision = RECRUITMENT_B; // save the winning process
      }
    }
  } else {

    /****************************************************/
    /* abandon                                          */
    /****************************************************/

    // TODO come sopra, due cose in parallelo e la somma delle prob non deve superare 1 altrimenti scoppia il macro

    /* leave immediately if reached the threshold */
    if(area_pop <= area_exploitation_threshold) {
      abandon = 255;
    }

    /* is the winning process? */
    extraction -= abandon;
    if(extraction <= 0) {
      extraction = 255; // put it back to 255 to avoid extracion of a second winner
      current_decision = ABANDON; // save the winning process
    }

    /****************************************************/
    /* cross inhibtion over a random agent              */
    /****************************************************/

    u_int8_t random = (rand_soft()*num_of_kbs)/255; // a random agent
    local_knowledge rand_agent = the_local_knowledge[random];
    // if the message is still valid
    if(rand_agent.timestamp - kilo_ticks < knowledge_not_valid_after) {
      cross = rand_agent.area_pop*k;
    }

    /* is the winning process? */
    extraction -= cross;
    if(extraction <= 0) {
      extraction = 255; // put it back to 255 to avoid extracion of a second winner
      if(rand_agent.state == COMMITTED_AREA_A) {
        current_decision = INHIBITION_A; // save the winning process
      } else {
        current_decision = INHIBITION_B; // save the winning process
      }
    }
  }

  /* Partially fill up the kb message  */
  /* the rest is filled up in the loop */
  interactive_message.data[0] = kilo_uid;
  /* send out area pop for interactive processes */
  interactive_message.data[6] = area_pop;
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
  // TODO use amaury random walk (better a levy than a brownian)
  switch( current_motion_type ) {
  case TURN_LEFT:
    if( kilo_ticks > last_decision_ticks + turning_ticks ) {
      /* start moving forward */
      last_decision_ticks = kilo_ticks;  // fixed time FORWARD
      //	last_decision_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
      set_motion(FORWARD);
    }
  case TURN_RIGHT:
    if( kilo_ticks > last_decision_ticks + turning_ticks ) {
      /* start moving forward */
      last_decision_ticks = kilo_ticks;  // fixed time FORWARD
      //	last_decision_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
      set_motion(FORWARD);
    }
    break;
  case FORWARD:
    if( kilo_ticks > last_decision_ticks + max_straight_ticks ) {
      /* perform a radnom turn */
      last_decision_ticks = kilo_ticks;
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
    uint8_t seed = rand_soft(); // use rand_soft that is faster
    rand_seed(seed);

    /* Initialise motion variables */
    last_decision_ticks=rand()%max_straight_ticks;
    set_motion(FORWARD);
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
  // TODO non prendere decisioni ad ogni step, prima devi prendere
  // informazioni sulle qualita'.
  // TODO ARK deve mandare informazioni solo sul piccolo cerchietto in cui e' il kb
  // e non su tutta la risorsa
  // TODO ARK deve vedere lo stato dei kbs dai led!

/*   // set color for area A red */
/*   set_color(RGB(3,0,0)); */
/*   // stop and exploit */
/*   set_motors(0,0); */
/*   set_motion(STOP); */
/* } else if(sa_type == INSIDE_AREA_B) { */
/*   // set color for area B green */
/*   set_color(RGB(0,3,0)); */
/*   // stop and exploit */
/*   set_motors(0,0); */
/*   set_motion(STOP); */
/*  } else if(sa_type == OUTSIDE_AREA) { */
/*   current_state = OUTSIDE_AREA; */
/*   // search for something */
/*   set_motion(FORWARD); */
/*   last_decision_ticks=kilo_ticks; */
/*  } */
/* } */


  // take decision only after exploration
  if(exploration_ticks <= kilo_ticks-last_decision_ticks) {
    take_decision();
    // reset last decision ticks
    last_decision_ticks = kilo_ticks;

    if(current_decision == COMMITTMENT) {
      // go to selected area
      // con piu' risorse metti un for
      if(current_state == INSIDE_AREA_A) {
        current_state = COMMITTED_AREA_A;
      } else if(current_state == INSIDE_AREA_B) {
        current_state = COMMITTED_AREA_B;
      }
      set_motion(STOP);

    } else if (current_decision == RECRUITMENT_A ||
               current_decision == RECRUITMENT_B) {
      // if recruited una volta reclutato vai in cerca dell'area A e finche' non la trovi non fai altro
      random_walk();

    } else if(current_decision == ABANDON){
      // leave current area
      current_state = NONE;
      random_walk();

    } else if(current_decision == INHIBITION_A ||
              current_decision == INHIBITION_B) {
      // leave the area
      random_walk();
    } else if(current_decision == NONE) {
      random_walk();
    }
  } else {
    random_walk();
  }

  /* fill up the rest of the message */
  /* fill up the message of uint8 by splitting the uint16 */
  interactive_message.data[2] = (my_coordinates.x >> 8); // hi part of the uint16
  interactive_message.data[3] = (my_coordinates.x & 0xff); // lo part of the uint16
  interactive_message.data[4] = (my_coordinates.y >> 8); // hi part of the uint16
  interactive_message.data[5] = (my_coordinates.y & 0xff); // lo part of the uint16

  /* fill up the current state */
  interactive_message.data[1] = current_state;
  /* fill up the crc */
  interactive_message.crc = message_crc(&interactive_message);
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

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
uint32_t last_turn_ticks = 0;
uint32_t turn_ticks = 60;
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;
uint32_t turn_into_random_walker_ticks = 160; /* timestep to wait without any direction message before turning into random_walker */
uint32_t last_direction_msg = 0;


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

/* Variables for Smart Arena messages */
int sa_type = 3; // 0,1,2 -> outside,in A, in B
int sa_payload = 0;
bool new_sa_msg = false; // true if there is a new message to unpack


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

/* pair of coordinates */
typedef struct arena_coordinates arena_coordinates;
struct arena_coordinates {
  float x;
  float y;
};

/* local knowledge about a kilobot */
typedef struct local_knowledge local_knowledge;
struct local_knowledge {
  int kb_id; // id of the kb to which the knowledge corresponds
  decision_t process; // its last process (used for interactions)
  arena_t state; // kilbots state (in whiche area?)
  arena_coordinates coordinates; // its location
};

/* current decions */
decision_t current_decision = NONE;
/* to share kilobots local knowledge */
message_t* interactive_message;


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
  //TODO DARIO keep as it is

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

void rx_message(message_t *msg, distance_measurement_t *d) {
  if(msg->crc == 0) {
    /* ----------------------------------*/
    // no valid message received
    /* ----------------------------------*/
    new_sa_msg = false;

  } else if (msg->type == 0) {

    /* ----------------------------------*/
    // parse ARK message
    /* ----------------------------------*/
    // unpack message, there are 3 kbs message in an ark messages
    // shift left and right and bitwise OR operator
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);

    if (id1 == kilo_uid) {
      // unpack type
      sa_type = msg->data[1] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
      new_sa_msg = true;
    }
    if (id2 == kilo_uid) {
      // unpack type
      sa_type = msg->data[4] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
      new_sa_msg = true;
    }
    if (id3 == kilo_uid) {
      // unpack type
      sa_type = msg->data[7] >> 2 & 0x0F;
      // unpack payload
      sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
      new_sa_msg = true;
    }

    if(new_sa_msg==true){
      // we do have a new message from ARK, check the payload
      if(current_state == OUTSIDE_AREA) {
        if((sa_type==1)) {
          current_state = INSIDE_AREA_A;
          // stop and exploit
          set_motors(0,0);
          set_motion(STOP);
          // set color for area A
          set_color(RGB(3,0,0));
        } else if (sa_type==2) {
          current_state = INSIDE_AREA_B;
          // stop and exploit
          set_motors(0,0);
          set_motion(STOP);
          // set color of area B
          set_color(RGB(0,3,0));
        }
      } else if((sa_type==0)) {
        current_state=OUTSIDE_AREA;
        // search for something
        set_motion(FORWARD);
        last_motion_ticks=kilo_ticks;
        // set color for uncommitted
        set_color(RGB(3,3,3));
      }
      new_sa_msg = false;
    }
  }
  /* } else if (msg->type == 1) { */
  /*   std::cout << " woooooooo msg type 1" << std::endl; */
  /*   /\* ----------------------------------*\/ */
  /*   // parse KB interactive message */
  /*   /\* ----------------------------------*\/ */

  /*   // custom data structure per byte */
  /*   // [ 1,  2, 3, 4, 5, 6, 7, 8, 9] */
  /*   // [id, id, x, x, y, y, r, c, s] */

  /*   // two bytes for the int id */
  /*   //TODO sta roba sicuro non funziona */
  /*   int id = (msg->data[0] << 2) | (msg->data[1] >> 6); */
  /*   // two bytes for the int position x */
  /*   int oth_x = (msg->data[2] << 2) | (msg->data[3] >> 6); */
  /*   // two bytes for the int position y */
  /*   int oth_y = (msg->data[4] << 2) | (msg->data[5] >> 6); */
  /*   // which process ID? */
  /*   unsigned char recruitment = msg->data[6]; */
  /*   unsigned char cross = msg->data[7]; */
  /*   unsigned char self = msg->data[8]; */
  /* } */
}


/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *tx_message() {
  /* this one is filled during the decision process */
  return &interactive_message;
}


/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision() {
  // no valid interactive process yet, set crc to 0
  interactive_message->crc = 0;

  // get current area utility if any

  // TODO decide using whatever knowledge we have and update the target
  // check status uncommitted or committed

  // if uncommitted
  // spontaneous commitment process through discovery
  // rectruitmet

  //if committed
  // spontaneous abandon process
  // cross-inhibition


  // fill up the message
  interactive_message;

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
    set_color(RGB(3,3,3));
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
  // take decision
  take_decision();

  if(current_decision == COMMITTMENT || current_decision == RECRUITMENT) {
    // TODO go to selected area ASAP
    random_walk();
  } else if(current_decision == ABANDONDONMENT || current_decision == INHIBITION) {
    // TODO leave the area ASAP
    random_walk();
  } else if(current_decision == NONE) {
    random_walk();
  }
}

int main() {
  kilo_init();
  kilo_message_rx = rx_message;
  kilo_start(setup, loop);

  return 0;
}

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


/* pair of coordinates used for selected resources */
struct target_area {
  float x;
  float y;
};

/* current decions */
decision_t current_decision = NONE;

/*-------------------------------------------------------------------*/
/* Other Variables                                                   */
/*-------------------------------------------------------------------*/

/* enum for different motion types */
typedef enum {
              FORWARD = 0,
              TURN_LEFT = 1,
              TURN_RIGHT = 2,
              STOP = 3,
} motion_t;

/* enum for boolean flags */
typedef enum {
              false = 0,
              true = 1,
} bool;

/* enum for the robot states */
typedef enum {
              OUTSIDE_AREA=0,
              INSIDE_AREA_A=1,
              INSIDE_AREA_B=2,
}action_t;

/* current motion type */
motion_t current_motion_type = STOP;

/* current state */
action_t current_state = OUTSIDE_AREA;

/* counters for motion, turning and random_walk */
uint32_t last_turn_ticks = 0;
uint32_t turn_ticks = 60;

unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;
uint32_t turn_into_random_walker_ticks = 160; /* timestep to wait without any direction message before turning into random_walker */
uint32_t last_direction_msg = 0;

/* Variables for Smart Arena messages */
int sa_type = 3;
int sa_payload = 0;
bool new_sa_msg = false;

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
/* message with 3 different kilobots ids.
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {
    if (msg->type == 0) {
        // unpack message
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

    } else if(msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }

    // we do have a new message, check the payload
    if(new_sa_msg==true){
      if(current_state == OUTSIDE_AREA) {
        if((sa_type==1)) {
          current_state = INSIDE_AREA_A;
          set_motors(0,0);
          set_motion(STOP);
          set_color(RGB(3,0,0));
        } else if (sa_type==2) {
          current_state = INSIDE_AREA_A;
          set_motors(0,0);
          set_motion(STOP);
          set_color(RGB(0,3,0));
        }
      } else if((sa_type==0) && (current_state==INSIDE_AREA_A || current_state==INSIDE_AREA_B)) {
        current_state=OUTSIDE_AREA;
        set_motion(FORWARD);
        last_motion_ticks=kilo_ticks;
        set_color(RGB(0,0,0));
      }
      new_sa_msg = false;
    }
}

/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/*-------------------------------------------------------------------*/
void take_decision() {
  //TODO decide using whatever knowledge we have and update the target
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
  //TODO DARIO keep as it is

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

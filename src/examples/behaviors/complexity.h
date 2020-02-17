#ifndef COMPLEXITY_H
#define COMPLEXITY_H

#ifdef ARGOS_simulator_BUILD

// this only works within ARGoS
#define DEBUG_KILOBOT

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#ifdef DEBUG_KILOBOT
typedef struct {
  uint8_t ema_resource0;
  uint8_t ema_resource1;
  uint8_t ema_resource2;
  uint8_t hits_empty;
  uint8_t hits_resource0;
  uint8_t hits_resource1;
  uint8_t hits_resource2;
  uint8_t num_messages;
} debug_info_t;
#endif

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif /* ARGOS_simulator_BUILD */

#endif /* COMPLEXITY_H */
